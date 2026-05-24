#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader
from aimdk_msgs.srv import SetMcInputSource


class ForceWalkOdom(Node):
    def __init__(self):
        super().__init__("force_walk_odom")

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
        self.start_xy = None

        self.sub = self.create_subscription(
            Odometry,
            "/aima/hal/odom/state",
            self.odom_cb,
            qos
        )

    def odom_cb(self, msg):
        self.odom = msg

    def wait_odom(self, seconds=3.0):
        print("等待 odom...")
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None:
                p = self.odom.pose.pose.position
                self.start_xy = (p.x, p.y)
                print(f"收到 odom: start x={p.x:.3f}, y={p.y:.3f}")
                return True
        print("没有收到 odom，但仍继续测试")
        return False

    def distance(self):
        if self.odom is None or self.start_xy is None:
            return 0.0
        p = self.odom.pose.pose.position
        dx = p.x - self.start_xy[0]
        dy = p.y - self.start_xy[1]
        return math.sqrt(dx * dx + dy * dy)

    def register_input_source(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print("等待 SetMcInputSource 服务...")

        # ADD / MODIFY / ENABLE 都试一遍，避免 node 已存在导致 ADD 失败
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

    def send_vel(self, forward, lateral, angular):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node"
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = float(lateral)
        msg.angular_velocity = float(angular)
        self.pub.publish(msg)

    def run(self):
        self.register_input_source()
        self.wait_odom()

        print("开始强制前进：forward=0.5 m/s，持续 6 秒")
        t0 = time.time()
        last_print = 0.0

        while time.time() - t0 < 6.0:
            self.send_vel(0.5, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)

            now = time.time()
            if now - last_print > 1.0:
                print(f"已走距离估计: {self.distance():.3f} m")
                last_print = now

            time.sleep(0.02)

        print("停止 2 秒")
        t0 = time.time()
        while time.time() - t0 < 2.0:
            self.send_vel(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

        print(f"最终 odom 距离估计: {self.distance():.3f} m")
        print("测试结束")


def main():
    rclpy.init()
    node = ForceWalkOdom()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
