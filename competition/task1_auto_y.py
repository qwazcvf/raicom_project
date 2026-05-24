#!/usr/bin/env python3
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader, RequestHeader, McActionCommand
from aimdk_msgs.srv import SetMcInputSource, SetMcAction


MODES = {
    "SD": "STAND_DEFAULT",
    "LD": "LOCOMOTION_DEFAULT",
}


class AutoYTask(Node):
    def __init__(self):
        super().__init__("task1_auto_y")

        self.pub = self.create_publisher(
            McLocomotionVelocity,
            "/aima/mc/locomotion/velocity",
            10,
        )

        self.input_client = self.create_client(
            SetMcInputSource,
            "/aimdk_5Fmsgs/srv/SetMcInputSource",
        )

        self.mode_client = self.create_client(
            SetMcAction,
            "/aimdk_5Fmsgs/srv/SetMcAction",
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.odom = None
        self.sub = self.create_subscription(
            Odometry,
            "/aima/hal/odom/state",
            self.odom_cb,
            qos,
        )

    def odom_cb(self, msg):
        self.odom = msg

    def pose(self):
        if self.odom is None:
            return None
        return self.odom.pose.pose.position

    def print_pose(self, label):
        p = self.pose()
        if p is None:
            print(f"{label}: no odom")
        else:
            print(f"{label}: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")

    def register_input_source(self):
        print("注册输入源 node...")
        while not self.input_client.wait_for_service(timeout_sec=1.0):
            print("等待 SetMcInputSource 服务...")

        ok = False
        for action in (1001, 1002, 2001):
            req = SetMcInputSource.Request()
            req.action.value = action
            req.input_source.name = "node"
            req.input_source.priority = 80
            req.input_source.timeout = 1000
            req.request.header.stamp = self.get_clock().now().to_msg()

            fut = self.input_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)

            if fut.done() and fut.result() is not None:
                resp = fut.result()
                code = resp.response.header.code
                state = resp.response.state.value
                print(f"register action={action}, code={code}, state={state}")
                if code == 0 or state in (1, 300, 400):
                    ok = True

        print("input source ok:", ok)
        return ok

    def set_mode(self, abbr):
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            print("等待 SetMcAction 服务...")

        req = SetMcAction.Request()
        req.header = RequestHeader()
        req.source = "node"
        req.command = McActionCommand()
        req.command.action_desc = MODES[abbr]

        for _ in range(8):
            req.header.stamp = self.get_clock().now().to_msg()
            fut = self.mode_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)

            if fut.done() and fut.result() is not None:
                resp = fut.result()
                status = resp.response.status.value
                msg = resp.response.message
                print(f"set mode {abbr}: status={status}, msg={msg}")
                if status == 1:
                    return True

        print(f"模式切换失败: {abbr}")
        return False

    def wait_odom(self, timeout=3.0):
        print("等待 odom...")
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None:
                self.print_pose("收到 odom")
                return True
        print("没有收到 odom")
        return False

    def send_vel(self, forward=0.0):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node"
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = 0.0
        msg.angular_velocity = 0.0
        self.pub.publish(msg)

    def stop_velocity(self, seconds=1.0):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.send_vel(0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def final_safe_stop(self):
        print("\n最终安全停车")
        self.stop_velocity(2.0)
        print("切回 SD 稳定站立")
        self.set_mode("SD")
        self.stop_velocity(2.0)
        self.print_pose("最终位置")
        print("任务完成")

    def pulse_test(self, forward, probe_time, fall_z):
        """
        小脉冲测试某个 forward 正负号。
        返回 dy，正数代表 y 增大，方向更接近交互区。
        """
        p0 = self.pose()
        if p0 is None:
            return -999.0

        y0 = p0.y
        z0 = p0.z

        print(f"\n标定测试 forward={forward:.3f}, start_y={y0:.3f}, start_z={z0:.3f}")

        t0 = time.time()
        while time.time() - t0 < probe_time:
            rclpy.spin_once(self, timeout_sec=0.01)

            p = self.pose()
            if p is not None and p.z < fall_z:
                print(f"标定中检测到可能摔倒 z={p.z:.3f}")
                self.stop_velocity(1.0)
                return -999.0

            self.send_vel(forward)
            time.sleep(0.02)

        self.stop_velocity(1.2)

        p1 = self.pose()
        if p1 is None:
            return -999.0

        dy = p1.y - y0
        print(f"标定结果 forward={forward:.3f}: end_y={p1.y:.3f}, dy={dy:.3f}, z={p1.z:.3f}")
        return dy

    def choose_forward_sign(self, base_speed, probe_time, fall_z):
        dy_pos = self.pulse_test(abs(base_speed), probe_time, fall_z)
        dy_neg = self.pulse_test(-abs(base_speed), probe_time, fall_z)

        print(f"\n标定汇总: +speed dy={dy_pos:.3f}, -speed dy={dy_neg:.3f}")

        if dy_pos <= 0 and dy_neg <= 0:
            print("两个方向都没有让 y 增大，停止。")
            return None

        if dy_pos >= dy_neg:
            chosen = abs(base_speed)
        else:
            chosen = -abs(base_speed)

        print(f"选择 forward={chosen:.3f}")
        return chosen

    def walk_to_y(self, target_y, forward, burst_time, rest_time, max_time, fall_z, backtrack_margin):
        start_time = time.time()
        segment = 0
        best_y = self.pose().y if self.pose() else -999.0

        while time.time() - start_time < max_time:
            rclpy.spin_once(self, timeout_sec=0.01)
            p = self.pose()

            if p is None:
                self.stop_velocity(0.2)
                continue

            if p.z < fall_z:
                print(f"\n检测到可能摔倒: z={p.z:.3f} < {fall_z:.3f}")
                return False

            if p.y > best_y:
                best_y = p.y

            if p.y < best_y - backtrack_margin:
                print(f"\n检测到明显后退: 当前 y={p.y:.3f}, 历史最好 y={best_y:.3f}")
                return False

            if p.y >= target_y:
                print(f"\n达到目标 y: 当前 y={p.y:.3f} >= target_y={target_y:.3f}")
                return True

            segment += 1
            print(f"\n第 {segment} 段前进，当前 y={p.y:.3f}, best_y={best_y:.3f}")

            t0 = time.time()
            last_print = 0.0

            while time.time() - t0 < burst_time:
                rclpy.spin_once(self, timeout_sec=0.01)
                p = self.pose()

                if p is not None:
                    if p.z < fall_z:
                        print(f"\n检测到可能摔倒: z={p.z:.3f} < {fall_z:.3f}")
                        self.stop_velocity(1.0)
                        return False

                    if p.y > best_y:
                        best_y = p.y

                    if p.y >= target_y:
                        print(f"\n达到目标 y: 当前 y={p.y:.3f}")
                        self.stop_velocity(1.0)
                        return True

                    if time.time() - last_print > 0.4:
                        print(f"\rx={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}", end="", flush=True)
                        last_print = time.time()

                self.send_vel(forward)
                time.sleep(0.02)

            print("\n小停顿，恢复姿态")
            self.stop_velocity(rest_time)

        print("\n达到最大时间，停止")
        return False

    def run(self, target_y, base_speed, probe_time, burst_time, rest_time, max_time, fall_z, backtrack_margin):
        print("===== task1_auto_y start =====")

        if not self.register_input_source():
            print("输入源失败")
            return

        print("切 SD，站稳")
        self.set_mode("SD")
        self.stop_velocity(1.0)
        time.sleep(2.0)

        if not self.wait_odom():
            return

        print("切 LD，进入行走模式")
        self.set_mode("LD")
        time.sleep(1.0)

        chosen = self.choose_forward_sign(base_speed, probe_time, fall_z)

        if chosen is None:
            self.final_safe_stop()
            return

        print(f"\n开始正式分段前进: target_y={target_y}, forward={chosen:.3f}")
        ok = self.walk_to_y(
            target_y=target_y,
            forward=chosen,
            burst_time=burst_time,
            rest_time=rest_time,
            max_time=max_time,
            fall_z=fall_z,
            backtrack_margin=backtrack_margin,
        )

        if ok:
            print("\n本次结果：成功到达目标 y")
        else:
            print("\n本次结果：保护停止或超时")

        self.final_safe_stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-y", type=float, default=-1.2)
    parser.add_argument("--base-speed", type=float, default=0.22)
    parser.add_argument("--probe-time", type=float, default=0.7)
    parser.add_argument("--burst-time", type=float, default=0.6)
    parser.add_argument("--rest-time", type=float, default=1.2)
    parser.add_argument("--max-time", type=float, default=35.0)
    parser.add_argument("--fall-z", type=float, default=0.52)
    parser.add_argument("--backtrack-margin", type=float, default=0.18)
    args = parser.parse_args()

    rclpy.init()
    node = AutoYTask()

    try:
        node.run(
            target_y=args.target_y,
            base_speed=args.base_speed,
            probe_time=args.probe_time,
            burst_time=args.burst_time,
            rest_time=args.rest_time,
            max_time=args.max_time,
            fall_z=args.fall_z,
            backtrack_margin=args.backtrack_margin,
        )
    except KeyboardInterrupt:
        print("\n手动中断")
        try:
            node.final_safe_stop()
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
