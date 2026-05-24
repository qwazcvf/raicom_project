#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aimdk_msgs.srv import GetMcMotions


class InspectMotions(Node):
    def __init__(self):
        super().__init__("inspect_mc_motions")
        self.cli = self.create_client(
            GetMcMotions,
            "/aimdk_5Fmsgs/srv/GetMcMotions"
        )

    def run(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print("等待 GetMcMotions 服务...")

        req = GetMcMotions.Request()

        # 尽量填 header，如果字段存在
        try:
            req.request.header.stamp = self.get_clock().now().to_msg()
        except Exception as e:
            print("header 填充跳过:", e)

        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

        if not fut.done() or fut.result() is None:
            print("GetMcMotions 调用失败")
            return

        resp = fut.result()
        print("===== raw response =====")
        print(resp)

        print("===== fields =====")
        for k, v in resp.get_fields_and_field_types().items():
            print(k, ":", v)

        print("===== dir response useful =====")
        for k in dir(resp):
            if not k.startswith("_"):
                try:
                    val = getattr(resp, k)
                    if not callable(val):
                        print(k, "=", val)
                except Exception:
                    pass


def main():
    rclpy.init()
    node = InspectMotions()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
