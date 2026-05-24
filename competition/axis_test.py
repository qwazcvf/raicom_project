#!/usr/bin/env python3
import argparse
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader
from aimdk_msgs.srv import SetMcInputSource


class AxisTest(Node):
    def __init__(self):
        super().__init__("axis_test")

        self.pub = self.create_publisher(
            McLocomotionVelocity,
            "/aima/mc/locomotion/velocity",
            10
        )

        self.cli = self.create_client(
            SetMcInputSource,
            "/aimdk_5Fmsgs/srv/SetMcInputSource"
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
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print("等待 SetMcInputSource 服务...")

        ok = False
        for action in (1001, 1002, 2001):
            req = SetMcInputSource.Request()
            req.action.value = action
            req.input_source.name = "node"
            req.input_source.priority = 80
            req.input_source.timeout = 1000
            req.request.header.stamp = self.get_clock().now().to_msg()

            fut = self.cli.call_async(req)
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

    def wait_odom(self):
        print("等待 odom...")
        t0 = time.time()
        while time.time() - t0 < 3.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None:
                p = self.odom.pose.pose.position
                print(f"start: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")
                return p.x, p.y
        print("没有收到 odom")
        return None, None

    def send_vel(self, f, l, a):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node"
        msg.forward_velocity = float(f)
        msg.lateral_velocity = float(l)
        msg.angular_velocity = float(a)
        self.pub.publish(msg)

    def run_test(self, f, l, a, duration):
        self.register_input_source()
        sx, sy = self.wait_odom()

        print(f"开始测试: forward={f}, lateral={l}, angular={a}, duration={duration}s")

        t0 = time.time()
        while time.time() - t0 < duration:
            self.send_vel(f, l, a)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

        print("停止")
        t0 = time.time()
        while time.time() - t0 < 2.0:
            self.send_vel(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

        if self.odom is not None and sx is not None:
            p = self.odom.pose.pose.position
            dx = p.x - sx
            dy = p.y - sy
            d = math.sqrt(dx * dx + dy * dy)
            print(f"end:   x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")
            print(f"delta: dx={dx:.3f}, dy={dy:.3f}, distance={d:.3f} m")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--forward", type=float, default=0.0)
    parser.add_argument("--lateral", type=float, default=0.0)
    parser.add_argument("--angular", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = AxisTest()
    node.run_test(args.forward, args.lateral, args.angular, args.duration)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
