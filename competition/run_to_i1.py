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
)
from aimdk_msgs.srv import SetMcInputSource, SetMcAction


MODES = {
    "SD": "STAND_DEFAULT",
    "LD": "LOCOMOTION_DEFAULT",
}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(q):
    # ROS 常用 yaw 计算
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RunToI1(Node):
    def __init__(self):
        super().__init__("run_to_i1")

        self.pub = self.create_publisher(
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

    def wait_odom(self, timeout=3.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None:
                p = self.odom.pose.pose.position
                yaw = yaw_from_quat(self.odom.pose.pose.orientation)
                print(f"当前 odom: x={p.x:.3f}, y={p.y:.3f}, yaw={yaw:.3f}")
                return True
        print("没有收到 odom")
        return False

    def register_input_source(self):
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
                status = resp.response.status.value
                print(f"set mode {abbr}: status={status}, msg={resp.response.message}")
                if status == 1:
                    return True

        return False

    def send_vel(self, forward, lateral, angular):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node"
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = float(lateral)
        msg.angular_velocity = float(angular)
        self.pub.publish(msg)

    def stop(self, seconds=1.5):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.send_vel(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def run(self, target_x, target_y, stop_radius, max_time):
        print("===== run_to_i1 start =====")

        self.register_input_source()

        print("切 SD 站稳")
        self.set_mode("SD")
        self.stop(1.0)
        time.sleep(2.0)

        self.wait_odom()

        print("切 LD 行走")
        self.set_mode("LD")
        time.sleep(1.0)

        print(f"目标点: x={target_x:.3f}, y={target_y:.3f}, stop_radius={stop_radius:.3f}")

        t0 = time.time()
        last_print = 0.0

        while time.time() - t0 < max_time:
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.odom is None:
                self.send_vel(0.0, 0.0, 0.0)
                time.sleep(0.02)
                continue

            p = self.odom.pose.pose.position
            q = self.odom.pose.pose.orientation

            dx = target_x - p.x
            dy = target_y - p.y
            dist = math.sqrt(dx * dx + dy * dy)

            yaw = yaw_from_quat(q)
            desired = math.atan2(dy, dx)
            err = norm_angle(desired - yaw)

            if dist <= stop_radius:
                print("\n已经接近交互I区中心，停车")
                break

            # 转向控制。角速度太小可能不生效，所以做最小幅度限制。
            angular = clamp(0.45 * err, -0.25, 0.25)
            if abs(angular) < 0.10:
                angular = 0.0

            # 误差大时先转向，误差小时再前进，避免边走边摔
            if abs(err) > 0.45:
                forward = 0.0
            elif abs(err) > 0.25:
                forward = 0.20
            else:
                forward = 0.28

            self.send_vel(forward, 0.0, angular)

            now = time.time()
            if now - last_print > 0.5:
                print(
                    f"\rx={p.x:.3f}, y={p.y:.3f}, dist={dist:.3f}, yaw={yaw:.2f}, "
                    f"desired={desired:.2f}, err={err:.2f}, f={forward:.2f}, a={angular:.2f}",
                    end="",
                    flush=True,
                )
                last_print = now

            time.sleep(0.02)

        print("\n停车")
        self.stop(2.0)

        print("切回 SD 稳定")
        self.set_mode("SD")
        self.stop(2.0)

        if self.odom is not None:
            p = self.odom.pose.pose.position
            print(f"最终位置: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")

        print("任务完成")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-x", type=float, default=0.0)
    parser.add_argument("--target-y", type=float, default=1.7)
    parser.add_argument("--stop-radius", type=float, default=0.18)
    parser.add_argument("--max-time", type=float, default=120.0)
    args = parser.parse_args()

    rclpy.init()
    node = RunToI1()

    try:
        node.run(args.target_x, args.target_y, args.stop_radius, args.max_time)
    except KeyboardInterrupt:
        node.stop(2.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
