#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAICOM 2026 智慧养老组 - 任务一入口

运行前请确保：
1. MuJoCo 仿真窗口已经启动；
2. mc/bin/em_run.sh 已经启动；
3. 当前在 Docker 容器环境内运行；
4. competition/ros2_overlay/install/setup.bash 存在。

本文件负责调用 run_tk_remote.sh，启动任务一遥控器。
"""

import os
import subprocess
import sys
from pathlib import Path


def main() -> int:
    here = Path(__file__).resolve().parent
    launcher = here / "run_tk_remote.sh"

    if not launcher.exists():
        print(f"ERROR: 找不到启动脚本: {launcher}", file=sys.stderr)
        return 1

    os.chmod(launcher, 0o755)
    return subprocess.call(["bash", str(launcher)], cwd=str(here))


if __name__ == "__main__":
    raise SystemExit(main())
