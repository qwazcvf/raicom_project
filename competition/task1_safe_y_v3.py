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
    "PD": "PASSIVE_DEFAULT",
}


class SafeYTaskV3(Node):
    def __init__(self):
        super().__init__("task1_safe_y_v3")

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

    def print_pose(self, label):
        if self.odom is None:
            print(f"{label}: no odom")
            return
        p = self.odom.pose.pose.position
        print(f"{label}: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")

    def final_safe_stop(self):
        print("\n最终安全停车")
        self.stop_velocity(2.0)

        print("切回 SD 稳定站立")
        self.set_mode("SD")
        self.stop_velocity(2.0)

        self.print_pose("最终位置")
        print("任务完成")

    def run(self, target_y, forward, burst_time, rest_time, max_time, fall_z, wrong_margin):
        print("===== task1_safe_y_v3 start =====")

        reached = False
        failed = False

        if not self.register_input_source():
            print("输入源失败，退出")
            return

        print("切 SD，站稳")
        self.set_mode("SD")
        self.stop_velocity(1.0)
        time.sleep(2.0)

        if not self.wait_odom():
            return

        start_y = self.odom.pose.pose.position.y
        print(f"起始 y={start_y:.3f}")

        print("切 LD，进入行走模式")
        self.set_mode("LD")
        time.sleep(1.0)

        print(
            f"开始：target_y={target_y}, forward={forward}, "
            f"burst_time={burst_time}, rest_time={rest_time}, "
            f"fall_z={fall_z}, wrong_margin={wrong_margin}"
        )

        start_time = time.time()
        segment = 0

        while time.time() - start_time < max_time:
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.odom is None:
                self.stop_velocity(0.2)
                continue

            p = self.odom.pose.pose.position

            if p.z < fall_z:
                print(f"\n检测到可能摔倒: z={p.z:.3f} < {fall_z:.3f}")
                failed = True
                break

            if p.y < start_y - wrong_margin:
                print(f"\n方向错误，y 变小太多: 当前 y={p.y:.3f}, 起始 y={start_y:.3f}")
                failed = True
                break

            if p.y >= target_y:
                print(f"\n达到目标 y: 当前 y={p.y:.3f} >= target_y={target_y:.3f}")
                reached = True
                break

            segment += 1
            print(f"\n第 {segment} 段前进")

            t0 = time.time()
            last_print = 0.0

            while time.time() - t0 < burst_time:
                rclpy.spin_once(self, timeout_sec=0.01)

                if self.odom is not None:
                    p = self.odom.pose.pose.position

                    if p.z < fall_z:
                        print(f"\n检测到可能摔倒: z={p.z:.3f} < {fall_z:.3f}")
                        failed = True
                        break

                    if p.y < start_y - wrong_margin:
                        print(f"\n方向错误，立即停止: 当前 y={p.y:.3f}, 起始 y={start_y:.3f}")
                        failed = True
                        break

                    if p.y >= target_y:
                        print(f"\n达到目标 y: 当前 y={p.y:.3f}")
                        reached = True
                        break

                    if time.time() - last_print > 0.4:
                        print(f"\rx={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}", end="", flush=True)
                        last_print = time.time()

                self.send_vel(forward)
                time.sleep(0.02)

            print("\n小停顿，恢复姿态")
            self.stop_velocity(rest_time)

            if reached or failed:
                break

        if reached:
            print("\n本次测试结果：成功到达目标 y")
        elif failed:
            print("\n本次测试结果：触发保护停止")
        else:
            print("\n本次测试结果：达到最大时间停止")

        self.final_safe_stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-y", type=float, default=-1.2)
    parser.add_argument("--forward", type=float, default=-0.20)
    parser.add_argument("--burst-time", type=float, default=0.6)
    parser.add_argument("--rest-time", type=float, default=1.2)
    parser.add_argument("--max-time", type=float, default=30.0)
    parser.add_argument("--fall-z", type=float, default=0.52)
    parser.add_argument("--wrong-margin", type=float, default=0.12)
    args = parser.parse_args()

    rclpy.init()
    node = SafeYTaskV3()

    try:
        node.run(
            target_y=args.target_y,
            forward=args.forward,
            burst_time=args.burst_time,
            rest_time=args.rest_time,
            max_time=args.max_time,
            fall_z=args.fall_z,
            wrong_margin=args.wrong_margin,
        )
    except KeyboardInterrupt:
        print("\n手动中断，安全停车")
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
