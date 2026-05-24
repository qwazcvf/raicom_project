#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import UpperBodyCommandArray, MessageHeader


class UpperBodySwing(Node):
    def __init__(self):
        super().__init__("upper_body_swing_simple")
        self.pub = self.create_publisher(
            UpperBodyCommandArray,
            "/mc/upper_body_command",
            10
        )
        self.seq = 0

    def publish_upper_body(self, arm_pos):
        msg = UpperBodyCommandArray()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "mc_upper_body"
        msg.header.sequence = self.seq
        self.seq += 1

        # 这个 source 必须按官方例程来
        msg.source = "remote_teleop_pc"

        # 0 = 不控制手，只控制头/臂
        msg.hand_sub_mode = 0
        msg.head_pos = [0.0, 0.0]
        msg.arm_pos = arm_pos
        msg.hand_pos = []

        self.pub.publish(msg)

    def run(self, duration, amp, freq, left_axis, right_axis):
        print("===== 上肢摆臂测试开始 =====")
        print(f"duration={duration}, amp={amp}, freq={freq}")
        print(f"left_axis={left_axis}, right_axis={right_axis}")
        print("如果动作太大或不对，按 Ctrl+C 停止")

        t0 = time.time()

        while time.time() - t0 < duration:
            t = time.time() - t0
            s = math.sin(2.0 * math.pi * freq * t)

            # arm_pos 一共 14 个数：前 7 个大概率左臂，后 7 个大概率右臂
            arm_pos = [0.0] * 14

            # 左右反相摆动：左手向前时，右手向后
            arm_pos[left_axis] = amp * s
            arm_pos[right_axis] = -amp * s

            self.publish_upper_body(arm_pos)

            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

        print("恢复上肢到 0 位")
        t1 = time.time()
        while time.time() - t1 < 1.0:
            self.publish_upper_body([0.0] * 14)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

        print("===== 上肢摆臂测试结束 =====")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--amp", type=float, default=0.35)
    parser.add_argument("--freq", type=float, default=0.45)

    # 默认先用 2 和 9，因为官方例程里第 3 个值 0.5 能让手臂明显变化
    parser.add_argument("--left-axis", type=int, default=2)
    parser.add_argument("--right-axis", type=int, default=9)

    args = parser.parse_args()

    rclpy.init()
    node = UpperBodySwing()

    try:
        node.run(
            duration=args.duration,
            amp=args.amp,
            freq=args.freq,
            left_axis=args.left_axis,
            right_axis=args.right_axis,
        )
    except KeyboardInterrupt:
        print("\n手动中断，恢复上肢")
        try:
            for _ in range(50):
                node.publish_upper_body([0.0] * 14)
                rclpy.spin_once(node, timeout_sec=0.01)
                time.sleep(0.02)
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
