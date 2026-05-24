#!/usr/bin/env python3
import time
import argparse
import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import UpperBodyCommandArray, MessageHeader


class UpperBodyAxisScan(Node):
    def __init__(self):
        super().__init__("upper_body_axis_scan")
        self.pub = self.create_publisher(
            UpperBodyCommandArray,
            "/mc/upper_body_command",
            10
        )
        self.seq = 0

    def publish_cmd(self, arm_pos):
        msg = UpperBodyCommandArray()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "mc_upper_body"
        msg.header.sequence = self.seq
        self.seq += 1

        msg.source = "remote_teleop_pc"

        # 关键：按官方 CLAW 模式来，不用 0
        msg.hand_sub_mode = 1
        msg.head_pos = [0.0, 0.0]
        msg.arm_pos = arm_pos
        msg.hand_pos = [1.0, 0.0]

        self.pub.publish(msg)

    def hold(self, arm_pos, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.publish_cmd(arm_pos)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def run(self, amp, hold_time):
        print("===== 开始扫描 arm_pos 14 个编号 =====")
        print("看 MuJoCo 里哪个编号会让左臂/右臂明显动")
        print("每个编号会先 +amp，再 -amp")
        print("如果动作危险，立刻 Ctrl+C")

        # 先回零
        self.hold([0.0] * 14, 1.0)

        for idx in range(14):
            print(f"\n>>> 测试 arm_pos[{idx}] = +{amp}")
            arm = [0.0] * 14
            arm[idx] = amp
            self.hold(arm, hold_time)

            print(f">>> 测试 arm_pos[{idx}] = -{amp}")
            arm = [0.0] * 14
            arm[idx] = -amp
            self.hold(arm, hold_time)

            print(">>> 回零")
            self.hold([0.0] * 14, 0.8)

        print("\n扫描结束，回零")
        self.hold([0.0] * 14, 1.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--amp", type=float, default=0.6)
    parser.add_argument("--hold-time", type=float, default=2.0)
    args = parser.parse_args()

    rclpy.init()
    node = UpperBodyAxisScan()

    try:
        node.run(args.amp, args.hold_time)
    except KeyboardInterrupt:
        print("\n手动中断，回零")
        try:
            node.hold([0.0] * 14, 1.0)
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
