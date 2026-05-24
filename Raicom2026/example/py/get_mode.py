import sys

import rclpy
from rclpy.node import Node

from aimdk_msgs.srv import GetMcAction
from aimdk_msgs.msg import CommonRequest


class GetModeClient(Node):
    def __init__(self):
        super().__init__("get_mode_client")
        self.client = self.create_client(GetMcAction, "/aimdk_5Fmsgs/srv/GetMcAction")

    def get_mode(self) -> dict:
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service not available")
            return {}

        req = GetMcAction.Request()
        req.request = CommonRequest()

        for i in range(3):
            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                break
            self.get_logger().info(f"Retrying... [{i}]")

        response = future.result()
        if response is None:
            self.get_logger().error("Service call failed")
            return {}

        return {
            "mode": response.info.action_desc,
            "status": response.info.status.value,
        }


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GetModeClient()
        result = node.get_mode()
        if result:
            print(f"Mode: {result['mode']}, Status: {result['status']}")
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
