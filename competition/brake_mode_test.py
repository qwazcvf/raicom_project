#!/usr/bin/env python3
import argparse
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader, RequestHeader, McActionCommand
from aimdk_msgs.srv import SetMcInputSource, SetMcAction


MODES = {
    "PD": "PASSIVE_DEFAULT",
    "DD": "DAMPING_DEFAULT",
    "JD": "JOINT_DEFAULT",
    "SD": "STAND_DEFAULT",
    "LD": "LOCOMOTION_DEFAULT",
}


class BrakeModeTest(Node):
    def __init__(self):
        super().__init__("brake_mode_test")

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
            rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)

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

    def wait_odom(self):
        print("等待 odom...")
        t0 = time.time()
        while time.time() - t0 < 3.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None:
                self.print_pose("收到 odom")
                return True
        return False

    def wait_basic_stable(self):
        print("等待基本站稳")
        t0 = time.time()
        while time.time() - t0 < 6.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose()
            if p and 0.62 <= p.z <= 0.70 and self.vxy() < 0.08:
                self.print_pose("基本站稳")
                return True
            if p:
                print(f"\rz={p.z:.3f}, vxy={self.vxy():.3f}", end="", flush=True)
        print("\n没站稳")
        self.print_pose("当前")
        return False

    def run(self, forward, walk_time, stop_mode):
        print("===== brake_mode_test start =====")
        print(f"forward={forward}, walk_time={walk_time}, stop_mode={stop_mode}")

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

        self.print_pose("走前")

        if not self.set_mode("LD"):
            print("LD 失败，不测试")
            return

        time.sleep(1.0)

        print("开始短距离走")
        t0 = time.time()
        while time.time() - t0 < walk_time:
            self.send_vel(forward)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

        self.print_pose("刹车前")

        print("先发 0 速度 0.2 秒")
        self.hold_zero(0.2)

        print(f"执行模式刹车：{stop_mode}")
        self.set_mode(stop_mode)

        print("观察 5 秒")
        t1 = time.time()
        last = 0.0
        while time.time() - t1 < 5.0:
            self.send_vel(0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            if time.time() - last > 0.5:
                self.print_pose("刹车观察")
                last = time.time()
            time.sleep(0.02)

        if stop_mode != "SD":
            print("最后切回 SD")
            self.set_mode("SD")
            self.hold_zero(2.0)

        self.print_pose("最终")
        print("测试结束")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--forward", type=float, default=0.55)
    parser.add_argument("--walk-time", type=float, default=1.2)
    parser.add_argument("--stop-mode", type=str, default="JD", choices=["JD", "DD", "SD"])
    args = parser.parse_args()

    rclpy.init()
    node = BrakeModeTest()

    try:
        node.run(args.forward, args.walk_time, args.stop_mode)
    except KeyboardInterrupt:
        print("\n手动中断")
        try:
            node.hold_zero(1.0)
            node.set_mode("SD")
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
