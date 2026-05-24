#!/usr/bin/env python3
import argparse
import math
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


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class AutoYTaskV4(Node):
    def __init__(self):
        super().__init__("task1_auto_y_v4")

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

    def yaw(self):
        if self.odom is None:
            return None
        return yaw_from_quat(self.odom.pose.pose.orientation)

    def print_pose(self, label):
        p = self.pose()
        y = self.yaw()
        if p is None:
            print(f"{label}: no odom")
        else:
            print(f"{label}: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, yaw={y:.3f}")

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

    def prepare_locomotion(self):
        print("切 SD，重新站稳")
        self.set_mode("SD")
        self.stop_velocity(1.0)
        time.sleep(1.5)
        self.wait_odom()

        print("切 LD，进入行走模式")
        self.set_mode("LD")
        time.sleep(0.8)

    def pulse_test(self, forward, probe_time, fall_z):
        """
        小脉冲测试某个 forward。
        返回 dy。dy > 0 才说明 y 往交互区方向变大。
        """
        self.prepare_locomotion()

        p0 = self.pose()
        if p0 is None:
            return -999.0

        y0 = p0.y
        print(f"\n标定测试 forward={forward:.3f}, start_y={y0:.3f}")

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

        self.stop_velocity(1.5)

        p1 = self.pose()
        if p1 is None:
            return -999.0

        dy = p1.y - y0
        print(f"标定结果 forward={forward:.3f}: end_y={p1.y:.3f}, dy={dy:.3f}, z={p1.z:.3f}")
        return dy

    def choose_forward(self, base_speed, probe_time, fall_z, min_dy):
        dy_pos = self.pulse_test(abs(base_speed), probe_time, fall_z)
        dy_neg = self.pulse_test(-abs(base_speed), probe_time, fall_z)

        print(f"\n标定汇总: +speed dy={dy_pos:.3f}, -speed dy={dy_neg:.3f}, min_dy={min_dy:.3f}")

        valid_pos = dy_pos >= min_dy
        valid_neg = dy_neg >= min_dy

        if not valid_pos and not valid_neg:
            print("两个方向都没有达到最小有效前进量，不进入正式行走。")
            return None

        if valid_pos and (not valid_neg or dy_pos >= dy_neg):
            chosen = abs(base_speed)
        else:
            chosen = -abs(base_speed)

        print(f"选择 forward={chosen:.3f}")
        return chosen

    def walk_to_y(self, target_y, forward, burst_time, rest_time, max_time, fall_z, min_segment_dy):
        self.prepare_locomotion()

        start_time = time.time()
        segment = 0
        reverse_count = 0

        while time.time() - start_time < max_time:
            rclpy.spin_once(self, timeout_sec=0.01)

            p = self.pose()
            if p is None:
                continue

            if p.z < fall_z:
                print(f"\n检测到可能摔倒: z={p.z:.3f} < {fall_z:.3f}")
                return False

            if p.y >= target_y:
                print(f"\n达到目标 y: 当前 y={p.y:.3f} >= target_y={target_y:.3f}")
                return True

            segment += 1
            seg_start_y = p.y
            print(f"\n第 {segment} 段前进: forward={forward:.3f}, start_y={seg_start_y:.3f}")

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

                    if p.y >= target_y:
                        print(f"\n达到目标 y: 当前 y={p.y:.3f}")
                        self.stop_velocity(1.0)
                        return True

                    if time.time() - last_print > 0.4:
                        print(f"\rx={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}", end="", flush=True)
                        last_print = time.time()

                self.send_vel(forward)
                time.sleep(0.02)

            self.stop_velocity(rest_time)

            p_after = self.pose()
            if p_after is None:
                continue

            seg_dy = p_after.y - seg_start_y
            print(f"\n本段结束: end_y={p_after.y:.3f}, seg_dy={seg_dy:.3f}")

            if seg_dy < -min_segment_dy:
                reverse_count += 1
                forward = -forward
                print(f"本段明显走反，切换 forward 符号，新 forward={forward:.3f}, reverse_count={reverse_count}")

                if reverse_count >= 3:
                    print("连续多次方向不稳定，保护停止")
                    return False

            elif seg_dy > min_segment_dy:
                reverse_count = 0

        print("\n达到最大时间，停止")
        return False

    def run(self, target_y, base_speed, probe_time, min_dy, burst_time, rest_time, max_time, fall_z, min_segment_dy):
        print("===== task1_auto_y_v4 start =====")

        if not self.register_input_source():
            print("输入源失败")
            return

        chosen = self.choose_forward(base_speed, probe_time, fall_z, min_dy)

        if chosen is None:
            self.final_safe_stop()
            return

        print(f"\n开始正式分段前进: target_y={target_y}, chosen_forward={chosen:.3f}")

        ok = self.walk_to_y(
            target_y=target_y,
            forward=chosen,
            burst_time=burst_time,
            rest_time=rest_time,
            max_time=max_time,
            fall_z=fall_z,
            min_segment_dy=min_segment_dy,
        )

        if ok:
            print("\n本次结果：成功到达目标 y")
        else:
            print("\n本次结果：保护停止或超时")

        self.final_safe_stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-y", type=float, default=-1.2)
    parser.add_argument("--base-speed", type=float, default=0.30)
    parser.add_argument("--probe-time", type=float, default=1.2)
    parser.add_argument("--min-dy", type=float, default=0.03)
    parser.add_argument("--burst-time", type=float, default=0.5)
    parser.add_argument("--rest-time", type=float, default=1.5)
    parser.add_argument("--max-time", type=float, default=35.0)
    parser.add_argument("--fall-z", type=float, default=0.52)
    parser.add_argument("--min-segment-dy", type=float, default=0.03)
    args = parser.parse_args()

    rclpy.init()
    node = AutoYTaskV4()

    try:
        node.run(
            target_y=args.target_y,
            base_speed=args.base_speed,
            probe_time=args.probe_time,
            min_dy=args.min_dy,
            burst_time=args.burst_time,
            rest_time=args.rest_time,
            max_time=args.max_time,
            fall_z=args.fall_z,
            min_segment_dy=args.min_segment_dy,
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
