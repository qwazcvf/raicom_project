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


class PulseWalk(Node):
    def __init__(self):
        super().__init__("task1_pulse_walk")

        self.vel_pub = self.create_publisher(
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

    def vxy(self):
        if self.odom is None:
            return 999.0
        v = self.odom.twist.twist.linear
        return math.sqrt(v.x * v.x + v.y * v.y)

    def print_pose(self, label):
        p = self.pose()
        if p is None:
            print(f"{label}: no odom")
        else:
            print(f"{label}: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, vxy={self.vxy():.3f}")

    def send_vel(self, forward):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node"
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = 0.0
        msg.angular_velocity = 0.0
        self.vel_pub.publish(msg)

    def hold_zero(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.send_vel(0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

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
                print(f"register action={action}, code={resp.response.header.code}, state={resp.response.state.value}")
                ok = True

        print("input source ok:", ok)
        return ok

    def set_mode(self, abbr):
        print(f"切 {abbr}")
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
            rclpy.spin_until_future_complete(self, fut, timeout_sec=0.6)

            if fut.done() and fut.result() is not None:
                resp = fut.result()
                status = resp.response.status.value
                msg = resp.response.message
                print(f"set mode {abbr}: status={status}, msg={msg}")
                if status == 1:
                    return True

            time.sleep(0.2)

        print(f"{abbr} 切换失败")
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

    def wait_basic_stable(self):
        print("等待基本站稳")
        stable_start = None
        t0 = time.time()

        while time.time() - t0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose()
            if p is None:
                continue

            ok = 0.62 <= p.z <= 0.70 and self.vxy() < 0.05

            if ok:
                if stable_start is None:
                    stable_start = time.time()
                if time.time() - stable_start > 1.5:
                    self.print_pose("确认站稳")
                    return True
            else:
                stable_start = None

            print(f"\rz={p.z:.3f}, vxy={self.vxy():.3f}, stable={ok}", end="", flush=True)

        print("\n没站稳，不走")
        self.print_pose("当前")
        return False

    def wait_really_stopped(self, timeout=8.0, v_threshold=0.035, drift_threshold=0.018, stable_time=1.2, min_z=0.50):
        print("等待真正停止...")
        stable_start = None
        ref_x = None
        ref_y = None
        t0 = time.time()
        last_print = 0.0

        while time.time() - t0 < timeout:
            self.send_vel(0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            p = self.pose()

            if p is None:
                time.sleep(0.02)
                continue

            if ref_x is None:
                ref_x = p.x
                ref_y = p.y

            drift = math.sqrt((p.x - ref_x) ** 2 + (p.y - ref_y) ** 2)
            v = self.vxy()
            stopped_now = (v < v_threshold) and (drift < drift_threshold) and (p.z > min_z)

            if stopped_now:
                if stable_start is None:
                    stable_start = time.time()
                    ref_x = p.x
                    ref_y = p.y

                if time.time() - stable_start >= stable_time:
                    self.print_pose("真正停止位置")
                    return True
            else:
                stable_start = None
                ref_x = p.x
                ref_y = p.y

            if time.time() - last_print > 0.5:
                print(
                    f"\r停止检测: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, "
                    f"vxy={v:.3f}, drift={drift:.3f}, stopped={stopped_now}",
                    end="",
                    flush=True,
                )
                last_print = time.time()

            time.sleep(0.02)

        print("\n停止超时")
        self.print_pose("超时时位置")
        return False

    def walk_pulse(self, forward, pulse_time, fall_z):
        print(f"发一次行走脉冲: forward={forward:.2f}, pulse_time={pulse_time:.2f}s")

        t0 = time.time()
        while time.time() - t0 < pulse_time:
            rclpy.spin_once(self, timeout_sec=0.01)
            p = self.pose()

            if p is not None and p.z < fall_z:
                print(f"\n脉冲中检测到可能摔倒: z={p.z:.3f} < {fall_z:.3f}")
                self.hold_zero(1.0)
                return False

            self.send_vel(forward)
            time.sleep(0.02)

        self.hold_zero(0.3)
        return True

    def run(self, target_y, forward, pulse_time, max_pulses, fall_z, tolerance):
        print("===== task1_pulse_walk start =====")
        print(f"target_y={target_y:.3f}, forward={forward:.2f}, pulse_time={pulse_time:.2f}")

        if not self.register_input_source():
            return

        if not self.set_mode("SD"):
            return

        self.hold_zero(1.0)
        time.sleep(2.0)

        if not self.wait_odom():
            return

        if not self.wait_basic_stable():
            return

        if not self.set_mode("LD"):
            print("LD 失败，不发速度")
            self.hold_zero(2.0)
            self.set_mode("SD")
            return

        time.sleep(1.0)

        last_y = None

        for i in range(1, max_pulses + 1):
            p = self.pose()
            if p is None:
                continue

            remain = target_y - p.y
            print(f"\n第 {i} 次脉冲前: y={p.y:.3f}, remain={remain:.3f}")

            if remain <= tolerance:
                print("已经到达目标附近")
                break

            # 接近目标时自动缩短脉冲
            if remain < 0.08:
                this_pulse = 0.35
            elif remain < 0.16:
                this_pulse = 0.60
            elif remain < 0.28:
                this_pulse = 0.85
            else:
                this_pulse = pulse_time

            ok = self.walk_pulse(forward, this_pulse, fall_z)
            if not ok:
                break

            stopped = self.wait_really_stopped(timeout=8.0)
            if not stopped:
                print("这次没等到真正停稳，停止任务")
                break

            p2 = self.pose()
            if p2 is None:
                break

            if last_y is not None:
                dy = p2.y - last_y
                print(f"本次停稳后推进 dy={dy:.3f}")
            last_y = p2.y

            if p2.y >= target_y - tolerance:
                print(f"到达目标附近: y={p2.y:.3f}, target={target_y:.3f}")
                break

        print("\n最终停车并切 SD")
        self.hold_zero(2.0)
        self.set_mode("SD")
        self.hold_zero(2.0)
        self.wait_really_stopped(timeout=5.0)

        self.print_pose("最终记录位置")
        print("任务完成")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-y", type=float, default=-1.0)
    parser.add_argument("--forward", type=float, default=0.55)
    parser.add_argument("--pulse-time", type=float, default=1.2)
    parser.add_argument("--max-pulses", type=int, default=12)
    parser.add_argument("--fall-z", type=float, default=0.50)
    parser.add_argument("--tolerance", type=float, default=0.04)
    args = parser.parse_args()

    rclpy.init()
    node = PulseWalk()

    try:
        node.run(
            target_y=args.target_y,
            forward=args.forward,
            pulse_time=args.pulse_time,
            max_pulses=args.max_pulses,
            fall_z=args.fall_z,
            tolerance=args.tolerance,
        )
    except KeyboardInterrupt:
        print("\n手动中断")
        try:
            node.hold_zero(2.0)
            node.set_mode("SD")
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
