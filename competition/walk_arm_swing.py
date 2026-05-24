#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import UpperBodyCommandArray, MessageHeader


class WalkArmSwing(Node):
    def __init__(self):
        super().__init__("walk_arm_swing")
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

        # 按官方 upper_body_command.py 的写法
        msg.source = "remote_teleop_pc"

        # 用官方 CLAW 模式，之前你测试这个是能动的
        msg.hand_sub_mode = 1
        msg.head_pos = [0.0, 0.0]
        msg.arm_pos = arm_pos
        msg.hand_pos = [1.0, 0.0]

        self.pub.publish(msg)

    def run(self, duration, shoulder_amp, elbow_amp, freq):
        print("===== 走路摆臂测试开始 =====")
        print(f"duration={duration}, shoulder_amp={shoulder_amp}, elbow_amp={elbow_amp}, freq={freq}")
        print("使用轴：左肩0，左肘3，右肩7，右肘10")
        print("如果动作太大、身体晃、方向不对，按 Ctrl+C")

        t0 = time.time()

        while time.time() - t0 < duration:
            t = time.time() - t0
            s = math.sin(2.0 * math.pi * freq * t)

            arm = [0.0] * 14

            # 左右肩反相：左臂向前时，右臂向后
            arm[0] = shoulder_amp * s
            arm[7] = -shoulder_amp * s

            # 肘部小幅配合，别太大，否则像甩胳膊
            arm[3] = elbow_amp * s
            arm[10] = -elbow_amp * s

            self.publish_upper_body(arm)

            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

        print("恢复上肢到 0 位")
        t1 = time.time()
        while time.time() - t1 < 1.0:
            self.publish_upper_body([0.0] * 14)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

        print("===== 走路摆臂测试结束 =====")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--shoulder-amp", type=float, default=0.45)
    parser.add_argument("--elbow-amp", type=float, default=0.12)
    parser.add_argument("--freq", type=float, default=0.55)
    args = parser.parse_args()

    rclpy.init()
    node = WalkArmSwing()

    try:
        node.run(
            duration=args.duration,
            shoulder_amp=args.shoulder_amp,
            elbow_amp=args.elbow_amp,
            freq=args.freq,
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
