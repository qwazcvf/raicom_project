#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node

from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader
from aimdk_msgs.srv import SetMcInputSource


class TestWalk(Node):
    def __init__(self):
        super().__init__("test_walk")
        self.pub = self.create_publisher(
            McLocomotionVelocity,
            "/aima/mc/locomotion/velocity",
            10
        )
        self.client = self.create_client(
            SetMcInputSource,
            "/aimdk_5Fmsgs/srv/SetMcInputSource"
        )

    def register_input_source(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("等待 SetMcInputSource 服务...")

        for action_value in (1001, 1002, 2001):
            req = SetMcInputSource.Request()
            req.action.value = action_value
            req.input_source.name = "node"
            req.input_source.priority = 40
            req.input_source.timeout = 1000

            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            if future.done() and future.result() is not None:
                resp = future.result()
                print("register:", action_value, "code:", resp.response.header.code, "state:", resp.response.state.value)
                return True

        return False

    def send_vel(self, forward, lateral, angular):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node"
        msg.forward_velocity = forward
        msg.lateral_velocity = lateral
        msg.angular_velocity = angular
        self.pub.publish(msg)

    def run(self):
        ok = self.register_input_source()
        print("input source ok:", ok)

        print("开始前进：forward=0.2，持续 2 秒")
        t0 = time.time()
        while time.time() - t0 < 2.0:
            self.send_vel(0.2, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

        print("停止")
        t0 = time.time()
        while time.time() - t0 < 1.0:
            self.send_vel(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)


def main():
    rclpy.init()
    node = TestWalk()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
