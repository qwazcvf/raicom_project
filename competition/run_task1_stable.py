#!/usr/bin/env python3
import time
import math
import argparse

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


class StableTask1(Node):
    def __init__(self):
        super().__init__("stable_task1")

        self.pub = self.create_publisher(
            McLocomotionVelocity,
            "/aima/mc/locomotion/velocity",
            10
        )

        self.input_client = self.create_client(
            SetMcInputSource,
            "/aimdk_5Fmsgs/srv/SetMcInputSource"
        )

        self.mode_client = self.create_client(
            SetMcAction,
            "/aimdk_5Fmsgs/srv/SetMcAction"
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
            qos
        )

    def odom_cb(self, msg):
        self.odom = msg

    def register_input_source(self):
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
                print(f"set mode {abbr}: status={resp.response.status.value}, msg={resp.response.message}")
                if resp.response.status.value == 1:
                    return True

        return False

    def wait_odom(self, timeout=3.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None:
                p = self.odom.pose.pose.position
                print(f"当前 odom: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")
                return True
        print("没有收到 odom")
        return False

    def send_vel(self, forward, lateral=0.0, angular=0.0):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node"
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = float(lateral)
        msg.angular_velocity = float(angular)
        self.pub.publish(msg)

    def stop(self, seconds=2.0):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.send_vel(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def walk_until_y(self, target_y, forward, angular, max_time):
        print(f"开始分段行走: target_y={target_y}, forward={forward}, angular={angular}, max_time={max_time}")

        t0 = time.time()
        last_print = 0.0

        while time.time() - t0 < max_time:
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.odom is not None:
                p = self.odom.pose.pose.position

                if time.time() - last_print > 0.5:
                    print(f"\rx={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}", end="", flush=True)
                    last_print = time.time()

                if p.y >= target_y:
                    print("\n达到本段 target_y")
                    break

            self.send_vel(forward, 0.0, angular)
            time.sleep(0.02)

        print("\n本段结束，先停一下")
        self.stop(1.0)

    def run(self, final_y):
        print("===== 稳定版任务一开始 =====")

        self.register_input_source()

        print("切 SD，站稳")
        self.set_mode("SD")
        self.stop(1.0)
        time.sleep(2.0)
        self.wait_odom()

        print("切 LD，准备走")
        self.set_mode("LD")
        time.sleep(1.0)

        # 第一段：轻微右转前进，别原地转
        # 注意：如果这一段发现它往墙边偏，后面我们把 angular 改成 +0.10
        self.walk_until_y(
            target_y=-0.30,
            forward=0.24,
            angular=-0.10,
            max_time=45.0
        )

        # 第二段：尽量直走，靠近交互I区
        self.walk_until_y(
            target_y=final_y,
            forward=0.24,
            angular=0.0,
            max_time=60.0
        )

        print("最终停车")
        self.stop(2.0)

        print("切回 SD 稳定站立")
        self.set_mode("SD")
        self.stop(2.0)

        if self.odom is not None:
            p = self.odom.pose.pose.position
            print(f"最终位置: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")

        print("任务完成")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--final-y", type=float, default=1.55)
    args = parser.parse_args()

    rclpy.init()
    node = StableTask1()

    try:
        node.run(args.final_y)
    except KeyboardInterrupt:
        print("\n手动中断，尝试停车")
        try:
            node.stop(1.0)
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
