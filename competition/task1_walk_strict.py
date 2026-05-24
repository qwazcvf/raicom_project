#!/usr/bin/env python3
import argparse
import time
import math

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


class StrictWalk(Node):
    def __init__(self):
        super().__init__("task1_walk_strict")

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

    def speed_xy(self):
        if self.odom is None:
            return 999.0
        v = self.odom.twist.twist.linear
        return math.sqrt(v.x * v.x + v.y * v.y)

    def print_pose(self, label):
        p = self.pose()
        if p is None:
            print(f"{label}: no odom")
        else:
            print(f"{label}: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, vxy={self.speed_xy():.3f}")

    def send_vel(self, forward):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node"
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = 0.0
        msg.angular_velocity = 0.0
        self.vel_pub.publish(msg)

    def hold_zero(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.send_vel(0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def ramp(self, start_v, end_v, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.01)
            ratio = (time.time() - t0) / seconds
            ratio = max(0.0, min(1.0, ratio))
            v = start_v + (end_v - start_v) * ratio
            self.send_vel(v)
            time.sleep(0.02)
        self.send_vel(end_v)

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

    def wait_strict_stable(self, z_low, z_high, stable_time, timeout):
        """
        严格站稳判定：
        1. z 必须在正常站立范围内，比如 0.62~0.69
        2. 水平速度不能太大
        3. x/y 漂移不能太大
        4. 持续 stable_time 秒
        """
        print(f"严格等待站稳: {z_low:.2f} <= z <= {z_high:.2f}, 持续 {stable_time:.1f}s")

        stable_start = None
        ref_x = None
        ref_y = None
        t0 = time.time()
        last_print = 0.0

        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose()
            if p is None:
                continue

            vxy = self.speed_xy()
            z_ok = z_low <= p.z <= z_high

            if stable_start is None:
                ref_x = p.x
                ref_y = p.y
                drift = 0.0
            else:
                drift = math.sqrt((p.x - ref_x) ** 2 + (p.y - ref_y) ** 2)

            stable_now = z_ok and vxy < 0.04 and drift < 0.025

            if stable_now:
                if stable_start is None:
                    stable_start = time.time()
                    ref_x = p.x
                    ref_y = p.y

                if time.time() - stable_start >= stable_time:
                    self.print_pose("确认站稳")
                    return True
            else:
                stable_start = None
                ref_x = p.x
                ref_y = p.y

            if time.time() - last_print > 0.3:
                print(
                    f"\rz={p.z:.3f}, vxy={vxy:.3f}, drift={drift:.3f}, stable={stable_now}",
                    end="",
                    flush=True,
                )
                last_print = time.time()

        print("\n没有达到严格站稳条件，不走。请 Reset 后重试。")
        self.print_pose("当前姿态")
        return False

    def run(self, final_y, forward, brake_before, max_time, fall_z):
        print("===== task1_walk_strict start =====")
        brake_y = final_y - brake_before

        print(f"final_y={final_y:.3f}, brake_y={brake_y:.3f}, forward={forward:.3f}")

        if not self.register_input_source():
            return

        if not self.set_mode("SD"):
            return

        self.hold_zero(1.0)
        time.sleep(2.0)

        if not self.wait_odom():
            return

        # 正常站稳大概 z=0.65。z 太高也不算稳定。
        if not self.wait_strict_stable(z_low=0.62, z_high=0.69, stable_time=2.0, timeout=12.0):
            self.hold_zero(2.0)
            return

        p0 = self.pose()
        start_y = p0.y
        print(f"\n起点 y={start_y:.3f}")

        if not self.set_mode("LD"):
            print("LD 没切成功，不发速度。")
            self.hold_zero(2.0)
            self.set_mode("SD")
            return

        time.sleep(1.0)

        print("开始走，到提前刹车点立即减速")
        t0 = time.time()
        last_print = 0.0
        cmd = 0.0

        while time.time() - t0 < max_time:
            rclpy.spin_once(self, timeout_sec=0.01)
            p = self.pose()
            if p is None:
                continue

            if p.z < fall_z:
                print(f"\n检测到可能摔倒: z={p.z:.3f} < {fall_z:.3f}")
                break

            if p.y >= brake_y:
                print(f"\n到提前刹车点: y={p.y:.3f} >= brake_y={brake_y:.3f}")
                break

            if p.y < start_y - 0.25:
                print(f"\n方向明显错误: 当前 y={p.y:.3f}, 起点 y={start_y:.3f}")
                break

            elapsed = time.time() - t0
            if elapsed < 1.0:
                cmd = forward * elapsed / 1.0
            else:
                cmd = forward

            self.send_vel(cmd)

            if time.time() - last_print > 0.4:
                print(f"\rx={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, cmd={cmd:.2f}", end="", flush=True)
                last_print = time.time()

            time.sleep(0.02)

        print("\n刹车：快速但不急停")
        self.ramp(cmd, 0.0, 0.8)

        print("LD 零速度保持 3 秒，等它自己稳住")
        self.hold_zero(3.0)

        print("切回 SD")
        self.set_mode("SD")
        self.hold_zero(2.0)

        self.print_pose("最终位置")
        print("任务完成")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--final-y", type=float, default=-1.0)
    parser.add_argument("--forward", type=float, default=0.55)
    parser.add_argument("--brake-before", type=float, default=0.45)
    parser.add_argument("--max-time", type=float, default=18.0)
    parser.add_argument("--fall-z", type=float, default=0.50)
    args = parser.parse_args()

    rclpy.init()
    node = StrictWalk()

    try:
        node.run(
            final_y=args.final_y,
            forward=args.forward,
            brake_before=args.brake_before,
            max_time=args.max_time,
            fall_z=args.fall_z,
        )
    except KeyboardInterrupt:
        print("\n手动中断，停车")
        try:
            node.ramp(0.3, 0.0, 0.8)
            node.set_mode("SD")
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
