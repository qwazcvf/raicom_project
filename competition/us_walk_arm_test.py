#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from aimdk_msgs.msg import (
    McLocomotionVelocity,
    MessageHeader,
    RequestHeader,
    McActionCommand,
    UpperBodyCommandArray,
)
from aimdk_msgs.srv import SetMcInputSource, SetMcAction


MODES = {
    "SD": "STAND_DEFAULT",
    "US": "UPPERBODY_REMOTE_SPLIT",
}


class USWalkArmTest(Node):
    def __init__(self):
        super().__init__("us_walk_arm_test")

        self.vel_pub = self.create_publisher(
            McLocomotionVelocity,
            "/aima/mc/locomotion/velocity",
            10,
        )

        self.arm_pub = self.create_publisher(
            UpperBodyCommandArray,
            "/mc/upper_body_command",
            10,
        )

        self.input_client = self.create_client(
            SetMcInputSource,
            "/aimdk_5Fmsgs/srv/SetMcInputSource",
        )

        self.mode_client = self.create_client(
            SetMcAction,
            "/aimdk_5Fmsgs/srv/SetMcAction",
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
            qos,
        )

        self.arm_seq = 0

    def odom_cb(self, msg):
        self.odom = msg

    def pose(self):
        if self.odom is None:
            return None
        return self.odom.pose.pose.position

    def print_pose(self, label):
        p = self.pose()
        if p is None:
            print(f"{label}: no odom")
        else:
            print(f"{label}: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")

    def register_input_source(self):
        print("注册输入源 node...")
        while not self.input_client.wait_for_service(timeout_sec=1.0):
            print("等待 SetMcInputSource 服务...")

        ok = False
        for action in (1001, 1002, 2001):
            req = SetMcInputSource.Request()
            req.action.value = action
            req.input_source.name = "node"
            req.input_source.priority = 80
            req.input_source.timeout = 1000
            req.request.header.stamp = self.get_clock().now().to_msg()

            fut = self.input_client.call_async(req)
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

    def set_mode(self, abbr):
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            print("等待 SetMcAction 服务...")

        req = SetMcAction.Request()
        req.header = RequestHeader()
        req.source = "node"
        req.command = McActionCommand()
        req.command.action_desc = MODES[abbr]

        for _ in range(8):
            req.header.stamp = self.get_clock().now().to_msg()
            fut = self.mode_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)

            if fut.done() and fut.result() is not None:
                resp = fut.result()
                print(f"set mode {abbr}: status={resp.response.status.value}, msg={resp.response.message}")
                if resp.response.status.value == 1:
                    return True

        print(f"模式切换失败: {abbr}")
        return False

    def wait_odom(self):
        print("等待 odom...")
        t0 = time.time()
        while time.time() - t0 < 3.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None:
                self.print_pose("收到 odom")
                return True
        print("没有收到 odom")
        return False

    def send_velocity(self, forward):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node"
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = 0.0
        msg.angular_velocity = 0.0
        self.vel_pub.publish(msg)

    def publish_arm(self, t, shoulder_amp, elbow_amp, freq):
        s = math.sin(2.0 * math.pi * freq * t)

        arm = [0.0] * 14
        arm[0] = shoulder_amp * s
        arm[7] = -shoulder_amp * s
        arm[3] = elbow_amp * s
        arm[10] = -elbow_amp * s

        msg = UpperBodyCommandArray()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "mc_upper_body"
        msg.header.sequence = self.arm_seq
        self.arm_seq += 1

        msg.source = "remote_teleop_pc"
        msg.hand_sub_mode = 1
        msg.head_pos = [0.0, 0.0]
        msg.arm_pos = arm
        msg.hand_pos = [1.0, 0.0]

        self.arm_pub.publish(msg)

    def stop_all(self, seconds=1.5):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.send_velocity(0.0)
            self.publish_arm(0.0, 0.0, 0.0, 0.35)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def run(self, forward, duration, shoulder_amp, elbow_amp, freq, fall_z):
        print("===== US 模式行走 + 摆臂测试 =====")

        self.register_input_source()

        print("先切 SD 站稳")
        self.set_mode("SD")
        self.stop_all(1.0)
        time.sleep(1.5)
        self.wait_odom()

        p0 = self.pose()
        if p0 is None:
            return
        start_x, start_y = p0.x, p0.y

        print("切 US 上肢远程控制模式")
        self.set_mode("US")
        time.sleep(1.0)

        print(f"开始测试：forward={forward}, duration={duration}")
        print("观察：1. 手臂是否摆动；2. 身体是否整体移动；3. 是否要摔")

        t0 = time.time()
        last_print = 0.0

        while time.time() - t0 < duration:
            now = time.time() - t0
            rclpy.spin_once(self, timeout_sec=0.01)

            p = self.pose()
            if p is not None:
                if p.z < fall_z:
                    print(f"\n检测到可能摔倒 z={p.z:.3f} < {fall_z}")
                    break

                if time.time() - last_print > 0.5:
                    print(f"\rx={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}", end="", flush=True)
                    last_print = time.time()

            self.send_velocity(forward)
            self.publish_arm(now, shoulder_amp, elbow_amp, freq)
            time.sleep(0.02)

        print("\n停止")
        self.stop_all(2.0)

        p1 = self.pose()
        if p1 is not None:
            print(f"起点: x={start_x:.3f}, y={start_y:.3f}")
            print(f"终点: x={p1.x:.3f}, y={p1.y:.3f}, z={p1.z:.3f}")
            print(f"位移: dx={p1.x - start_x:.3f}, dy={p1.y - start_y:.3f}")

        print("切回 SD")
        self.set_mode("SD")
        self.stop_all(2.0)
        print("测试结束")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--forward", type=float, default=-0.30)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--shoulder-amp", type=float, default=0.20)
    parser.add_argument("--elbow-amp", type=float, default=0.04)
    parser.add_argument("--freq", type=float, default=0.35)
    parser.add_argument("--fall-z", type=float, default=0.52)
    args = parser.parse_args()

    rclpy.init()
    node = USWalkArmTest()

    try:
        node.run(
            forward=args.forward,
            duration=args.duration,
            shoulder_amp=args.shoulder_amp,
            elbow_amp=args.elbow_amp,
            freq=args.freq,
            fall_z=args.fall_z,
        )
    except KeyboardInterrupt:
        print("\n手动中断")
        try:
            node.stop_all(1.0)
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
