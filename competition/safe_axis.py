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


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SafeAxis(Node):
    def __init__(self):
        super().__init__("safe_axis")
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

    def register(self):
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
                print(f"register action={action}, code={resp.response.header.code}, state={resp.response.state.value}")
                if resp.response.header.code == 0 or resp.response.state.value in (1, 300, 400):
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
                yaw = yaw_from_quat(self.odom.pose.pose.orientation)
                print(f"start: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, yaw={yaw:.3f}")
                return p.x, p.y, p.z
        print("没有收到 odom")
        return None, None, None

    def send_vel(self, f, l, a):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node"
        msg.forward_velocity = float(f)
        msg.lateral_velocity = float(l)
        msg.angular_velocity = float(a)
        self.pub.publish(msg)

    def stop(self, seconds=1.5):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.send_vel(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def run(self, f, l, a, duration):
        self.register()
        sx, sy, sz = self.wait_odom()

        print(f"开始安全测试: forward={f}, lateral={l}, angular={a}, duration={duration}")

        t0 = time.time()
        last_print = 0.0

        while time.time() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.odom is not None:
                p = self.odom.pose.pose.position
                yaw = yaw_from_quat(self.odom.pose.pose.orientation)

                if p.z < 0.45:
                    print(f"\n检测到可能摔倒 z={p.z:.3f}，立即停止")
                    break

                if time.time() - last_print > 0.5:
                    print(f"\rx={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, yaw={yaw:.3f}", end="", flush=True)
                    last_print = time.time()

            self.send_vel(f, l, a)
            time.sleep(0.02)

        print("\n停止")
        self.stop(2.0)

        if self.odom is not None and sx is not None:
            p = self.odom.pose.pose.position
            yaw = yaw_from_quat(self.odom.pose.pose.orientation)
            dx = p.x - sx
            dy = p.y - sy
            print(f"end:   x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, yaw={yaw:.3f}")
            print(f"delta: dx={dx:.3f}, dy={dy:.3f}, distance={(dx*dx+dy*dy)**0.5:.3f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--forward", type=float, default=0.0)
    parser.add_argument("--lateral", type=float, default=0.0)
    parser.add_argument("--angular", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = SafeAxis()
    try:
        node.run(args.forward, args.lateral, args.angular, args.duration)
    except KeyboardInterrupt:
        print("\n手动中断，停车")
        node.stop(1.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
