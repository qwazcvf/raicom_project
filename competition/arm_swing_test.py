#!/usr/bin/env python3
import math
import time
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from aimdk_msgs.msg import JointCommandArray, JointCommand, JointStateArray


ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
]


class ArmSwingTest(Node):
    def __init__(self):
        super().__init__("arm_swing_test")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.arm_state = None

        self.sub = self.create_subscription(
            JointStateArray,
            "/aima/hal/joint/arm/state",
            self.arm_state_cb,
            qos,
        )

        self.pub = self.create_publisher(
            JointCommandArray,
            "/aima/hal/joint/arm/command",
            10,
        )

        self.base = {}

    def arm_state_cb(self, msg):
        self.arm_state = msg

    def wait_arm_state(self, timeout=3.0):
        print("等待 arm state...")
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.arm_state is not None and len(self.arm_state.joints) > 0:
                self.base = {}
                for j in self.arm_state.joints:
                    self.base[j.name] = j.position
                print("收到 arm state，关节数量:", len(self.base))
                return True
        print("没有收到 arm state")
        return False

    def get_base(self, name):
        return self.base.get(name, 0.0)

    def publish_arm(self, t, amp):
        """
        只做小幅度反相摆臂。
        不动腿，不控制腰，不控制头。
        """
        phase = math.sin(2.0 * math.pi * 0.55 * t)

        cmd = JointCommandArray()

        for name in ARM_JOINTS:
            j = JointCommand()
            j.name = name
            j.position = self.get_base(name)
            j.velocity = 0.0
            j.effort = 0.0
            j.stiffness = 12.0
            j.damping = 1.5

            # 左右肩 pitch 反相摆动
            if name == "left_shoulder_pitch_joint":
                j.position = self.get_base(name) + amp * phase
            elif name == "right_shoulder_pitch_joint":
                j.position = self.get_base(name) - amp * phase

            # 手肘轻微配合，幅度更小
            elif name == "left_elbow_joint":
                j.position = self.get_base(name) - 0.35 * amp * phase
            elif name == "right_elbow_joint":
                j.position = self.get_base(name) + 0.35 * amp * phase

            cmd.joints.append(j)

        self.pub.publish(cmd)

    def hold_base(self, seconds=1.0):
        t0 = time.time()
        while time.time() - t0 < seconds:
            cmd = JointCommandArray()
            for name in ARM_JOINTS:
                j = JointCommand()
                j.name = name
                j.position = self.get_base(name)
                j.velocity = 0.0
                j.effort = 0.0
                j.stiffness = 12.0
                j.damping = 1.5
                cmd.joints.append(j)
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def run(self, duration, amp):
        if not self.wait_arm_state():
            return

        print(f"开始手臂摆动测试：duration={duration}, amp={amp}")
        print("如果手臂抖得厉害或机器人要倒，立刻 Ctrl+C")

        t0 = time.time()
        while time.time() - t0 < duration:
            now = time.time() - t0
            self.publish_arm(now, amp)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

        print("恢复到初始手臂姿态")
        self.hold_base(1.0)
        print("测试结束")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--amp", type=float, default=0.15)
    args = parser.parse_args()

    rclpy.init()
    node = ArmSwingTest()

    try:
        node.run(args.duration, args.amp)
    except KeyboardInterrupt:
        print("\n手动中断")
        try:
            node.hold_base(1.0)
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
