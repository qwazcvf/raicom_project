#!/usr/bin/env python3
import argparse
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
    "PD": "PASSIVE_DEFAULT",
}


class SafeForwardTask(Node):
    def __init__(self):
        super().__init__("task1_safe_forward")

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
        print("等待 odom...")
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None:
                p = self.odom.pose.pose.position
                print(f"收到 odom: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")
                return True
        print("没有收到 odom，停止测试")
        return False

    def register_input_source(self):
        print("注册输入源 node...")
        while not self.input_client.wait_for_service(timeout_sec=1.0):
            print("等待 SetMcInputSource 服务...")

        ok = False

        # 1001 ADD；1002 MODIFY；2001 ENABLE
        # ADD 可能因为已经注册过而返回 code=1，所以继续尝试 MODIFY/ENABLE
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
        if abbr not in MODES:
            print(f"未知模式: {abbr}")
            return False

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
                msg = resp.response.message
                print(f"set mode {abbr}: status={status}, msg={msg}")

                if status == 1:
                    return True

        print(f"模式切换失败: {abbr}")
        return False

    def send_vel(self, forward=0.0, lateral=0.0, angular=0.0):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node"
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = float(lateral)
        msg.angular_velocity = float(angular)
        self.pub.publish(msg)

    def stop_velocity(self, seconds=1.0):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.send_vel(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def is_fallen(self, fall_z):
        if self.odom is None:
            return False
        z = self.odom.pose.pose.position.z
        return z < fall_z

    def print_pose(self, prefix="pose"):
        if self.odom is None:
            return
        p = self.odom.pose.pose.position
        print(f"{prefix}: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")

    def walk_burst(self, forward, burst_time, fall_z):
        """
        小段前进。每段内部持续发布速度。
        如果检测到 z 太低，立即停车。
        """
        t0 = time.time()
        last_print = 0.0

        while time.time() - t0 < burst_time:
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.odom is not None:
                p = self.odom.pose.pose.position

                if p.z < fall_z:
                    print(f"\n检测到可能摔倒: z={p.z:.3f} < {fall_z:.3f}")
                    self.stop_velocity(1.0)
                    return False

                if time.time() - last_print > 0.5:
                    print(f"\r走路中: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}", end="", flush=True)
                    last_print = time.time()

            self.send_vel(forward, 0.0, 0.0)
            time.sleep(0.02)

        print("")
        return True

    def run(self, target_y, forward, burst_time, rest_time, max_time, fall_z):
        print("===== task1_safe_forward start =====")

        if not self.register_input_source():
            print("输入源注册失败，退出")
            return

        print("切 SD，先站稳")
        self.set_mode("SD")
        self.stop_velocity(1.0)
        time.sleep(2.0)

        if not self.wait_odom():
            return

        self.print_pose("起始位置")

        print("切 LD，进入行走模式")
        self.set_mode("LD")
        time.sleep(1.0)

        print(
            f"开始安全分段前进: target_y={target_y}, forward={forward}, "
            f"burst_time={burst_time}, rest_time={rest_time}, max_time={max_time}, fall_z={fall_z}"
        )

        start_time = time.time()
        segment = 0

        while time.time() - start_time < max_time:
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.odom is None:
                self.stop_velocity(0.2)
                continue

            p = self.odom.pose.pose.position

            if p.z < fall_z:
                print(f"\n检测到机器人可能已经摔倒: z={p.z:.3f}")
                break

            if p.y >= target_y:
                print(f"\n达到目标 y: 当前 y={p.y:.3f} >= target_y={target_y:.3f}")
                break

            segment += 1
            print(f"\n第 {segment} 段前进")
            ok = self.walk_burst(forward, burst_time, fall_z)

            print("小停顿，恢复姿态")
            self.stop_velocity(rest_time)

            if not ok:
                break

        print("\n最终停车")
        self.stop_velocity(2.0)

        print("切回 SD 稳定站立")
        self.set_mode("SD")
        self.stop_velocity(2.0)

        self.print_pose("最终位置")
        print("任务完成")


def main():
    parser = argparse.ArgumentParser()

    # 第一次建议 target_y 不要太大，先跑到 -0.8 或 -0.5 测稳定
    parser.add_argument("--target-y", type=float, default=-0.8)

    # 目前你实测 0.3 能走，但长期可能晃；先用 0.22 或 0.24 更稳
    parser.add_argument("--forward", type=float, default=0.22)

    # 分段走：每次走 1.2 秒，然后停 0.8 秒
    parser.add_argument("--burst-time", type=float, default=1.2)
    parser.add_argument("--rest-time", type=float, default=0.8)

    parser.add_argument("--max-time", type=float, default=60.0)

    # 正常站立 z 大约 0.65；摔倒后会接近 0.1
    parser.add_argument("--fall-z", type=float, default=0.45)

    args = parser.parse_args()

    rclpy.init()
    node = SafeForwardTask()

    try:
        node.run(
            target_y=args.target_y,
            forward=args.forward,
            burst_time=args.burst_time,
            rest_time=args.rest_time,
            max_time=args.max_time,
            fall_z=args.fall_z,
        )
    except KeyboardInterrupt:
        print("\n手动中断，尝试停车")
        try:
            node.stop_velocity(1.0)
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
