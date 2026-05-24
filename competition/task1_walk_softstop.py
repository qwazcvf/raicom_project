#!/usr/bin/env python3
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader, RequestHeader, McActionCommand
from aimdk_msgs.srv import SetMcInputSource, SetMcAction


MODES = {
    "SD": "STAND_DEFAULT",
    "LD": "LOCOMOTION_DEFAULT",
}


class SoftStopWalk(Node):
    def __init__(self):
        super().__init__("task1_walk_softstop")

        self.vel_pub = self.create_publisher(
            McLocomotionVelocity,
            "/aima/mc/locomotion/velocity",
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

    def send_vel(self, forward):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node"
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = 0.0
        msg.angular_velocity = 0.0
        self.vel_pub.publish(msg)

    def hold_velocity(self, forward, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.send_vel(forward)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def ramp_velocity(self, start_v, end_v, seconds):
        print(f"平滑变速: {start_v:.2f} -> {end_v:.2f}, {seconds:.1f}s")
        t0 = time.time()

        while time.time() - t0 < seconds:
            ratio = (time.time() - t0) / seconds
            ratio = max(0.0, min(1.0, ratio))
            v = start_v + (end_v - start_v) * ratio
            self.send_vel(v)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

        self.send_vel(end_v)

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
        print(f"切 {abbr}")
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
            rclpy.spin_until_future_complete(self, fut, timeout_sec=0.7)

            if fut.done() and fut.result() is not None:
                resp = fut.result()
                status = resp.response.status.value
                msg = resp.response.message
                print(f"set mode {abbr}: status={status}, msg={msg}")
                if status == 1:
                    return True

            time.sleep(0.3)

        print(f"模式切换失败: {abbr}")
        return False

    def wait_odom(self, timeout=3.0):
        print("等待 odom...")
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None:
                self.print_pose("收到 odom")
                return True
        print("没有收到 odom")
        return False

    def wait_stable(self, min_z=0.62, seconds=1.0, timeout=8.0):
        print(f"等待站稳: z >= {min_z:.2f} 持续 {seconds:.1f}s")
        stable_start = None
        t0 = time.time()

        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose()

            if p is None:
                continue

            if p.z >= min_z:
                if stable_start is None:
                    stable_start = time.time()
                if time.time() - stable_start >= seconds:
                    self.print_pose("站稳位置")
                    return True
            else:
                stable_start = None

            print(f"\rz={p.z:.3f}", end="", flush=True)

        print("\n没有站稳")
        self.print_pose("当前姿态")
        return False

    def run(self, target_y, forward, max_time, fall_z, slow_distance):
        print("===== task1_walk_softstop start =====")

        if not self.register_input_source():
            return

        if not self.set_mode("SD"):
            return

        self.hold_velocity(0.0, 1.0)
        time.sleep(2.0)

        if not self.wait_odom():
            return

        if not self.wait_stable():
            return

        p0 = self.pose()
        start_y = p0.y
        print(f"起点 y={start_y:.3f}, 目标 y={target_y:.3f}, forward={forward:.3f}")

        if not self.set_mode("LD"):
            print("LD 没切成功，不发速度")
            self.hold_velocity(0.0, 2.0)
            self.set_mode("SD")
            return

        time.sleep(1.0)

        print("开始平滑加速行走")
        t0 = time.time()
        last_print = 0.0
        current_cmd = 0.0

        while time.time() - t0 < max_time:
            rclpy.spin_once(self, timeout_sec=0.01)
            p = self.pose()

            if p is None:
                continue

            if p.z < fall_z:
                print(f"\n检测到可能摔倒: z={p.z:.3f} < {fall_z:.3f}")
                break

            remain = target_y - p.y

            if remain <= 0.0:
                print(f"\n达到目标附近: y={p.y:.3f}")
                break

            if p.y < start_y - 0.20:
                print(f"\n方向错误: 当前 y={p.y:.3f}, 起点 y={start_y:.3f}")
                break

            elapsed = time.time() - t0

            if elapsed < 1.2:
                desired = forward * elapsed / 1.2
            elif remain < slow_distance:
                # 接近目标提前减速，最低保持 0.22，避免速度太低不走
                ratio = max(0.0, min(1.0, remain / slow_distance))
                desired = 0.22 + (forward - 0.22) * ratio
            else:
                desired = forward

            current_cmd = desired
            self.send_vel(current_cmd)

            if time.time() - last_print > 0.4:
                print(
                    f"\rx={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, remain={remain:.3f}, cmd={current_cmd:.2f}",
                    end="",
                    flush=True,
                )
                last_print = time.time()

            time.sleep(0.02)

        print("\n开始柔和停车，不直接急刹")
        self.ramp_velocity(current_cmd, 0.22 if current_cmd > 0 else -0.22, 1.0)
        self.hold_velocity(0.22 if current_cmd > 0 else -0.22, 0.6)
        self.ramp_velocity(0.22 if current_cmd > 0 else -0.22, 0.0, 2.5)

        print("LD 下保持零速度 2 秒")
        self.hold_velocity(0.0, 2.0)

        print("再切回 SD")
        self.set_mode("SD")
        self.hold_velocity(0.0, 2.0)

        self.print_pose("最终位置")
        print("任务完成")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-y", type=float, default=-1.0)
    parser.add_argument("--forward", type=float, default=0.55)
    parser.add_argument("--max-time", type=float, default=20.0)
    parser.add_argument("--fall-z", type=float, default=0.50)
    parser.add_argument("--slow-distance", type=float, default=0.35)
    args = parser.parse_args()

    rclpy.init()
    node = SoftStopWalk()

    try:
        node.run(
            target_y=args.target_y,
            forward=args.forward,
            max_time=args.max_time,
            fall_z=args.fall_z,
            slow_distance=args.slow_distance,
        )
    except KeyboardInterrupt:
        print("\n手动中断，柔和停车")
        try:
            node.ramp_velocity(0.22, 0.0, 1.5)
            node.set_mode("SD")
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
