#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import pty
import select
import signal
import subprocess
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import messagebox

# ==============================================================================
# RAICOM Tk 遥控器：美化 UI + 已验证可用的 keyboard.py 代理控制逻辑
# 只依赖 Python 标准库；不修改官方 keyboard.py。
# ==============================================================================

KEYS = {
    "w": b"w",
    "s": b"s",
    "q": b"q",
    "e": b"e",
    "space": b" ",
    "esc": b"\x1b",
}

BASE = "/home/agi/x2_deploy_workspace"
COMP = f"{BASE}/competition"
LOG_DIR = f"{COMP}/logs"
SET_MODE_DIR = f"{BASE}/Raicom2026/example/py"

CANDIDATE_KEYBOARD_DIRS = [
    f"{BASE}/Raicom2026/aimdk-aarch64-1bde262f-artifacts/src/py_examples/py_examples",
    f"{BASE}/aimdk-aarch64-1bde262f-artifacts/src/py_examples/py_examples",
]

# 官方 keyboard.py 通常每按一次 w，forward 增加约 0.2；每按一次 q/e，angular 改变约 0.2
# 为了比赛稳定，默认只给 2 档前进和 1 档旋转。需要改可在运行前设置环境变量。
MAX_FORWARD_LEVEL = int(os.environ.get("MAX_FORWARD_LEVEL", "2"))
MAX_YAW_LEVEL = int(os.environ.get("MAX_YAW_LEVEL", "1"))
STEP = float(os.environ.get("REMOTE_STEP", "0.2"))


def now():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def find_keyboard_dir():
    env_dir = os.environ.get("KEYBOARD_DIR", "")
    if env_dir and os.path.exists(os.path.join(env_dir, "keyboard.py")):
        return env_dir

    for d in CANDIDATE_KEYBOARD_DIRS:
        if os.path.exists(os.path.join(d, "keyboard.py")):
            return d

    for root, dirs, files in os.walk(BASE):
        if "keyboard.py" in files and "py_examples" in root:
            return root

    raise FileNotFoundError("找不到官方 keyboard.py，请设置 KEYBOARD_DIR=/path/to/py_examples")


class KeyboardProxy:
    """
    通过伪终端启动官方 keyboard.py，并向它发送 w/s/q/e/space。
    这样不改官方文件，只把键盘操作封装成图形遥控器。
    """

    def __init__(self, keyboard_dir):
        os.makedirs(LOG_DIR, exist_ok=True)
        self.keyboard_dir = keyboard_dir
        self.log_path = os.path.join(LOG_DIR, f"tk_remote_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
        self.log_fp = open(self.log_path, "ab", buffering=0)
        self.master_fd = None
        self.proc = None
        self.lock = threading.RLock()
        self.forward_level = 0
        self.yaw_level = 0

    def log(self, msg):
        line = f"[{now()}] {msg}\n"
        print(line, end="", flush=True)
        self.log_fp.write(line.encode("utf-8", errors="ignore"))

    def start_keyboard(self):
        keyboard_py = os.path.join(self.keyboard_dir, "keyboard.py")
        if not os.path.exists(keyboard_py):
            raise FileNotFoundError(keyboard_py)

        master_fd, slave_fd = pty.openpty()
        self.master_fd = master_fd

        self.proc = subprocess.Popen(
            "python3 keyboard.py",
            cwd=self.keyboard_dir,
            shell=True,
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            preexec_fn=os.setsid,
            close_fds=True,
        )
        os.close(slave_fd)
        self.log(f"started keyboard.py pid={self.proc.pid}, dir={self.keyboard_dir}")
        threading.Thread(target=self.reader_loop, daemon=True).start()
        time.sleep(1.0)

    def reader_loop(self):
        while True:
            if self.proc is None:
                return
            if self.proc.poll() is not None:
                self.log(f"keyboard.py exited code={self.proc.returncode}")
                return
            try:
                r, _, _ = select.select([self.master_fd], [], [], 0.2)
                if not r:
                    continue
                data = os.read(self.master_fd, 4096)
                if not data:
                    return
                self.log_fp.write(data)
            except OSError as exc:
                self.log(f"reader stopped: {exc}")
                return
            except Exception as exc:
                self.log(f"reader error: {exc}")
                return

    def send_key(self, key, repeat=1, interval=0.08):
        if key not in KEYS:
            raise ValueError(f"unknown key: {key}")
        if self.proc is None or self.proc.poll() is not None:
            raise RuntimeError("keyboard.py 未运行")
        for _ in range(repeat):
            os.write(self.master_fd, KEYS[key])
            time.sleep(interval)

    def set_levels(self, forward_level, yaw_level):
        """
        forward_level: 0..MAX_FORWARD_LEVEL
        yaw_level: -MAX_YAW_LEVEL..MAX_YAW_LEVEL
        yaw_level > 0 表示右转，对应官方 e；yaw_level < 0 表示左转，对应官方 q。
        """
        with self.lock:
            forward_level = max(0, min(MAX_FORWARD_LEVEL, int(forward_level)))
            yaw_level = max(-MAX_YAW_LEVEL, min(MAX_YAW_LEVEL, int(yaw_level)))

            df = forward_level - self.forward_level
            dy = yaw_level - self.yaw_level

            if df > 0:
                self.send_key("w", repeat=df)
            elif df < 0:
                self.send_key("s", repeat=-df)

            if dy > 0:
                self.send_key("e", repeat=dy)
            elif dy < 0:
                self.send_key("q", repeat=-dy)

            self.forward_level = forward_level
            self.yaw_level = yaw_level

            self.log(
                f"set forward_level={self.forward_level}, yaw_level={self.yaw_level}, "
                f"forward≈{self.forward_level * STEP:.2f} m/s, yaw≈{self.yaw_level * STEP:.2f} rad/s"
            )

    def stop(self):
        with self.lock:
            try:
                self.send_key("space", repeat=3, interval=0.10)
            except Exception as exc:
                self.log(f"stop send failed: {exc}")
            self.forward_level = 0
            self.yaw_level = 0
            self.log("STOP: space x3")

    def set_mode(self, mode):
        cmd = f"cd {SET_MODE_DIR} && python3 set_mode.py {mode}"
        self.log(f"run mode command: {cmd}")
        result = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.log(result.stdout.strip())
        return result.returncode, result.stdout

    def close(self):
        self.stop()

        try:
            if self.proc is not None and self.proc.poll() is None:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                time.sleep(0.5)
        except Exception:
            pass

        try:
            if self.proc is not None and self.proc.poll() is None:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
        except Exception:
            pass

        try:
            if self.master_fd is not None:
                os.close(self.master_fd)
        except Exception:
            pass

        try:
            self.log_fp.close()
        except Exception:
            pass


class JoystickCanvas(tk.Canvas):
    """自定义摇杆组件，只依赖 Tkinter 标准库绘制。"""

    def __init__(self, master, axis="y", command=None, bg_color="#24283B", width=150, height=150):
        super().__init__(master, width=width, height=height, bg=bg_color, highlightthickness=0)
        self.axis = axis
        self.command = command
        self.width = width
        self.height = height
        self.center_x = self.width // 2
        self.center_y = self.height // 2

        self.bg_radius = min(self.width, self.height) // 2 - 25
        self.knob_radius = 16

        self.create_oval(
            self.center_x - self.bg_radius,
            self.center_y - self.bg_radius,
            self.center_x + self.bg_radius,
            self.center_y + self.bg_radius,
            fill="#1A1B26",
            outline="#414868",
            width=3,
        )

        self.create_line(
            self.center_x,
            self.center_y - self.bg_radius + 5,
            self.center_x,
            self.center_y + self.bg_radius - 5,
            fill="#414868",
            dash=(4, 4),
        )
        self.create_line(
            self.center_x - self.bg_radius + 5,
            self.center_y,
            self.center_x + self.bg_radius - 5,
            self.center_y,
            fill="#414868",
            dash=(4, 4),
        )

        knob_color = "#7AA2F7" if axis == "y" else "#FF9E64"
        self.knob = self.create_oval(
            self.center_x - self.knob_radius,
            self.center_y - self.knob_radius,
            self.center_x + self.knob_radius,
            self.center_y + self.knob_radius,
            fill=knob_color,
            outline="#24283B",
            width=2,
        )

        self.bind("<ButtonPress-1>", self.handle)
        self.bind("<B1-Motion>", self.handle)
        self.bind("<ButtonRelease-1>", self.release)

    def handle(self, event):
        x = event.x
        y = event.y
        dx = x - self.center_x
        dy = y - self.center_y

        if self.axis == "y":
            dx = 0
        elif self.axis == "x":
            dy = 0

        dist = (dx**2 + dy**2) ** 0.5
        if dist > self.bg_radius and dist != 0:
            dx = dx * self.bg_radius / dist
            dy = dy * self.bg_radius / dist

        self.coords(
            self.knob,
            self.center_x + dx - self.knob_radius,
            self.center_y + dy - self.knob_radius,
            self.center_x + dx + self.knob_radius,
            self.center_y + dy + self.knob_radius,
        )

        if self.command:
            if self.axis == "y":
                # 向上推为正前进；向下不后退，直接归零，比赛更稳。
                level_norm = max(0.0, -dy / self.bg_radius)
            else:
                # 左负右正。
                level_norm = dx / self.bg_radius
            self.command(level_norm)

    def reset_knob(self):
        self.coords(
            self.knob,
            self.center_x - self.knob_radius,
            self.center_y - self.knob_radius,
            self.center_x + self.knob_radius,
            self.center_y + self.knob_radius,
        )

    def release(self, event):
        self.reset_knob()
        if self.command:
            self.command(0.0)


class App:
    def __init__(self, root, proxy):
        self.root = root
        self.proxy = proxy
        self.root.title("@wzc-virtual-machine_Raicom2026")

        self.root.geometry("620x390")
        self.root.configure(bg="#1A1B26")
        self.root.resizable(True, True)

        self.forward_level = 0
        self.yaw_level = 0

        self.build_ui()
        self.setup_bindings()

        self.root.protocol("WM_DELETE_WINDOW", self.quit)

    def build_ui(self):
        top_frame = tk.Frame(self.root, bg="#1A1B26")
        top_frame.pack(side=tk.TOP, fill=tk.X, padx=12, pady=8)

        tk.Label(
            top_frame,
            text="RAICOM 遥控台",
            font=("Helvetica", 17, "bold"),
            bg="#1A1B26",
            fg="#C0CAF5",
        ).pack(side=tk.LEFT)

        self.status_label = tk.Label(
            top_frame,
            text="状态: Connected | Mode: Unknown",
            font=("Helvetica", 9),
            bg="#1A1B26",
            fg="#9ECE6A",
        )
        self.status_label.pack(side=tk.RIGHT, pady=10)

        mid_frame = tk.Frame(self.root, bg="#1A1B26")
        mid_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8)

        left_panel = tk.Frame(mid_frame, bg="#24283B", highlightbackground="#414868", highlightthickness=1)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=3)

        tk.Label(
            left_panel,
            text="FORWARD 前进",
            font=("Helvetica", 10, "bold"),
            bg="#24283B",
            fg="#7AA2F7",
        ).pack(pady=(15, 5))

        self.fwd_val_label = tk.Label(
            left_panel,
            text="0.00 m/s",
            font=("Courier", 14, "bold"),
            bg="#24283B",
            fg="#FFFFFF",
        )
        self.fwd_val_label.pack()

        self.fwd_canvas = JoystickCanvas(left_panel, axis="y", command=self.on_fwd_joy)
        self.fwd_canvas.pack(pady=(4, 6))

        right_panel = tk.Frame(mid_frame, bg="#24283B", highlightbackground="#414868", highlightthickness=1)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=3)

        tk.Label(
            right_panel,
            text="ANGULAR 旋转",
            font=("Helvetica", 10, "bold"),
            bg="#24283B",
            fg="#FF9E64",
        ).pack(pady=(15, 5))

        self.ang_val_label = tk.Label(
            right_panel,
            text="0.00 rad/s",
            font=("Courier", 14, "bold"),
            bg="#24283B",
            fg="#FFFFFF",
        )
        self.ang_val_label.pack()

        self.ang_canvas = JoystickCanvas(right_panel, axis="x", command=self.on_ang_joy)
        self.ang_canvas.pack(pady=(4, 6))

        bot_frame = tk.Frame(self.root, bg="#1A1B26")
        bot_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=8)

        btn_font = ("Helvetica", 9, "bold")

        tk.Button(
            bot_frame,
            text="SD 稳定站立",
            font=btn_font,
            bg="#3D59A1",
            fg="#FFFFFF",
            activebackground="#7AA2F7",
            activeforeground="#FFFFFF",
            borderwidth=0,
            width=10,
            height=1,
            cursor="hand2",
            command=lambda: self.set_mode("SD"),
        ).pack(side=tk.LEFT, padx=5)

        tk.Button(
            bot_frame,
            text="LD 行走模式",
            font=btn_font,
            bg="#9ECE6A",
            fg="#1A1B26",
            activebackground="#B5E877",
            activeforeground="#1A1B26",
            borderwidth=0,
            width=10,
            height=1,
            cursor="hand2",
            command=lambda: self.set_mode("LD"),
        ).pack(side=tk.LEFT, padx=5)

        tk.Button(
            bot_frame,
            text="退出 EXIT",
            font=btn_font,
            bg="#565F89",
            fg="#FFFFFF",
            activebackground="#414868",
            activeforeground="#FFFFFF",
            borderwidth=0,
            width=8,
            height=1,
            cursor="hand2",
            command=self.quit,
        ).pack(side=tk.RIGHT, padx=5)

        tk.Button(
            bot_frame,
            text="急停 E-STOP",
            font=("Helvetica", 10, "bold"),
            bg="#F7768E",
            fg="#1A1B26",
            activebackground="#FF9E64",
            activeforeground="#1A1B26",
            borderwidth=0,
            width=11,
            height=1,
            cursor="hand2",
            command=self.stop,
        ).pack(side=tk.RIGHT, padx=15)

        self.log_label = tk.Label(
            bot_frame,
            text=f"Log: {self.proxy.log_path}",
            font=("Courier", 6),
            bg="#1A1B26",
            fg="#565F89",
            anchor="w",
        )
        self.log_label.pack(side=tk.LEFT, padx=18)

    def setup_bindings(self):
        self.root.bind("<w>", lambda e: self.change_forward(1))
        self.root.bind("<s>", lambda e: self.change_forward(-1))
        self.root.bind("<q>", lambda e: self.change_yaw(-1))
        self.root.bind("<e>", lambda e: self.change_yaw(1))
        self.root.bind("<x>", lambda e: self.change_forward(0, reset=True))
        self.root.bind("<c>", lambda e: self.change_yaw(0, reset=True))
        self.root.bind("<space>", lambda e: self.stop())

    def on_fwd_joy(self, norm):
        level = int(round(norm * MAX_FORWARD_LEVEL))
        self.change_forward(level, absolute=True)

    def on_ang_joy(self, norm):
        level = int(round(norm * MAX_YAW_LEVEL))
        self.change_yaw(level, absolute=True)

    def _update_labels(self):
        fwd_speed = self.forward_level * STEP
        yaw_speed = self.yaw_level * STEP
        self.fwd_val_label.config(text=f"{fwd_speed:.2f} m/s")
        self.ang_val_label.config(text=f"{yaw_speed:.2f} rad/s")

    def _apply_levels(self):
        self.forward_level = max(0, min(MAX_FORWARD_LEVEL, self.forward_level))
        self.yaw_level = max(-MAX_YAW_LEVEL, min(MAX_YAW_LEVEL, self.yaw_level))
        self._update_labels()
        self.proxy.set_levels(self.forward_level, self.yaw_level)

    def change_forward(self, val, absolute=False, reset=False):
        if reset:
            self.forward_level = 0
        elif absolute:
            self.forward_level = val
        else:
            self.forward_level += val
        self._apply_levels()

    def change_yaw(self, val, absolute=False, reset=False):
        if reset:
            self.yaw_level = 0
        elif absolute:
            self.yaw_level = val
        else:
            self.yaw_level += val
        self._apply_levels()

    def stop(self):
        self.forward_level = 0
        self.yaw_level = 0
        self.fwd_canvas.reset_knob()
        self.ang_canvas.reset_knob()
        self._update_labels()
        self.proxy.stop()

    def set_mode(self, mode):
        code, output = self.proxy.set_mode(mode)
        if code == 0:
            self.status_label.config(text=f"状态: Connected | Mode: {mode}")
        else:
            self.status_label.config(text=f"状态: Connected | Mode: {mode} failed")
            messagebox.showerror(f"{mode} 失败", output[-1200:])

    def quit(self):
        self.stop()
        self.proxy.close()
        self.root.quit()
        self.root.destroy()


def main():
    keyboard_dir = find_keyboard_dir()
    proxy = KeyboardProxy(keyboard_dir)
    proxy.start_keyboard()

    root = tk.Tk()
    root.option_add("*Font", "Helvetica 11")

    App(root, proxy)
    root.mainloop()


if __name__ == "__main__":
    main()
