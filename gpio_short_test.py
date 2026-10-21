#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
树莓派 Zero 2 W GPIO 焊点粘连测试（含完整自诊断）

功能：
1) 运行前自诊断：
   - 系统版本 / Python版本
   - RPi.GPIO 可用性与版本
   - 是否 root
   - /dev/gpiochip* 是否存在
   - raspi-gpio / pinctrl 命令是否可用
   - GPIO 最小读写自测（setup/input/output/cleanup）

2) 粘连检测：
   - 逐个 GPIO 作为输出，其他 GPIO 作为输入，检测是否跟随电平变化
   - 高电平轮询 + 低电平轮询两轮
   - 输出疑似短路的 GPIO 对（含物理引脚号）

注意：
- 运行前请断开所有外接设备/HAT/杜邦线，避免误判
- 本脚本主要检测 GPIO 与 GPIO 之间粘连
- 电源脚(3V3/5V/GND)短路请务必用万用表复核
"""

import os
import sys
import time
import glob
import shutil
import platform
import subprocess
from typing import Dict, List, Set, Tuple

# -----------------------------
# 配置区
# -----------------------------
ALL_GPIO: List[int] = [
    2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
    17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27
]
HW_PULLUP = {2, 3}  # I2C1 SDA/SCL 板载上拉
SETTLE = 0.008      # 电平稳定等待时间（秒）
SELFTEST_PIN = 17   # 最小读写自测引脚（可改）

BCM_TO_PHYSICAL: Dict[int, int] = {
    2: 3, 3: 5, 4: 7, 5: 29, 6: 31, 7: 26, 8: 24, 9: 21, 10: 19,
    11: 23, 12: 32, 13: 33, 14: 8, 15: 10, 16: 36, 17: 11, 18: 12,
    19: 35, 20: 38, 21: 40, 22: 15, 23: 16, 24: 18, 25: 22, 26: 37, 27: 13,
}


# -----------------------------
# 工具函数
# -----------------------------
def run_cmd(cmd: List[str], timeout: int = 5) -> Tuple[int, str, str]:
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout.strip(), p.stderr.strip()
    except Exception as e:
        return 999, "", str(e)


def read_os_release() -> Dict[str, str]:
    info = {}
    path = "/etc/os-release"
    if not os.path.exists(path):
        return info
    try:
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if "=" in line:
                    k, v = line.split("=", 1)
                    info[k] = v.strip('"')
    except Exception:
        pass
    return info


def print_header(title: str):
    print("\n" + "=" * 72)
    print(title)
    print("=" * 72)


def print_ok(msg: str):
    print(f"[ OK ] {msg}")


def print_warn(msg: str):
    print(f"[WARN] {msg}")


def print_err(msg: str):
    print(f"[ERR ] {msg}")


# -----------------------------
# 自诊断
# -----------------------------
def diagnostics() -> Tuple[bool, object]:
    print_header("1) 环境自诊断")

    os_info = read_os_release()
    pretty = os_info.get("PRETTY_NAME", "Unknown OS")
    print_ok(f"OS: {pretty}")
    print_ok(f"Kernel: {platform.release()}")
    print_ok(f"Python: {sys.version.split()[0]}")

    is_root = (os.geteuid() == 0)
    if is_root:
        print_ok("当前为 root 权限")
    else:
        print_warn("当前不是 root，建议用 sudo 运行（可能导致 GPIO 访问失败）")

    chips = sorted(glob.glob("/dev/gpiochip*"))
    if chips:
        print_ok(f"检测到 gpiochip 设备: {', '.join(chips)}")
    else:
        print_err("未发现 /dev/gpiochip*，GPIO 子系统可能不可用")

    for cmd in ["raspi-gpio", "pinctrl"]:
        if shutil.which(cmd):
            rc, out, err = run_cmd([cmd, "get", "17"] if cmd == "raspi-gpio" else [cmd, "17"], timeout=3)
            if rc == 0:
                print_ok(f"{cmd} 可用")
            else:
                print_warn(f"{cmd} 存在但调用返回非0（可忽略）: {err or out}")
        else:
            print_warn(f"{cmd} 不在 PATH 中（可选工具）")

    # 导入 RPi.GPIO
    try:
        import RPi.GPIO as GPIO  # type: ignore
        ver = getattr(GPIO, "VERSION", "unknown")
        print_ok(f"RPi.GPIO 导入成功，版本: {ver}")
    except Exception as e:
        print_err(f"RPi.GPIO 导入失败: {e}")
        print_err("请安装: sudo apt install python3-rpi.gpio")
        return False, None

    # 最小 GPIO 自测
    print_header("2) GPIO 最小读写自测")
    try:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # 输入下拉读取
        GPIO.setup(SELFTEST_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        time.sleep(SETTLE)
        v1 = GPIO.input(SELFTEST_PIN)
        print_ok(f"GPIO{SELFTEST_PIN} 输入下拉读取成功，值={v1}")

        # 输出高低
        GPIO.setup(SELFTEST_PIN, GPIO.OUT, initial=GPIO.LOW)
        time.sleep(SETTLE)
        GPIO.output(SELFTEST_PIN, GPIO.HIGH)
        time.sleep(SETTLE)
        GPIO.output(SELFTEST_PIN, GPIO.LOW)
        time.sleep(SETTLE)
        print_ok(f"GPIO{SELFTEST_PIN} 输出高/低切换成功")

        # 恢复输入
        GPIO.setup(SELFTEST_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.cleanup()
        print_ok("GPIO cleanup 成功")

    except Exception as e:
        print_err(f"GPIO 最小自测失败: {e}")
        msg = str(e).lower()
        if "xgpiohandlerequest" in msg or "invalid arguement" in msg or "invalid argument" in msg:
            print_err("检测到疑似底层 GPIO 接口参数错误（常见于系统与 RPi.GPIO 兼容问题）")
            print_warn("建议：")
            print_warn("  1) 确认使用 sudo 运行")
            print_warn("  2) 升级系统和 python3-rpi.gpio")
            print_warn("  3) 在 Bookworm 上考虑切换 libgpiod 版本脚本")
        return False, None

    return True, GPIO


# -----------------------------
# 粘连测试核心
# -----------------------------
def setup_all_input(GPIO, pull):
    for p in ALL_GPIO:
        GPIO.setup(p, GPIO.IN, pull_up_down=pull)


def test_drive(GPIO, level) -> Set[Tuple[int, int]]:
    pull = GPIO.PUD_DOWN if level == GPIO.HIGH else GPIO.PUD_UP
    shorts: Set[Tuple[int, int]] = set()

    for drv in ALL_GPIO:
        setup_all_input(GPIO, pull)
        time.sleep(SETTLE)

        idle = {p: GPIO.input(p) for p in ALL_GPIO}

        GPIO.setup(drv, GPIO.OUT, initial=level)
        time.sleep(SETTLE)

        for p in ALL_GPIO:
            if p == drv:
                continue

            # 高电平轮中，GPIO2/3 本身有硬件上拉，跳过“被拉高”判定
            if level == GPIO.HIGH and p in HW_PULLUP:
                continue

            val = GPIO.input(p)
            if val == level and idle[p] != level:
                shorts.add(tuple(sorted((drv, p))))

        GPIO.setup(drv, GPIO.IN, pull_up_down=pull)

    return shorts


def test_stuck_to_power_or_gnd(GPIO) -> List[str]:
    """
    粗略检测某些引脚是否疑似“被强拉”到电源或地：
    - 设为下拉输入却长期读高 -> 疑似粘 3V3
    - 设为上拉输入却长期读低 -> 疑似粘 GND
    """
    issues = []

    # 下拉观察
    for p in ALL_GPIO:
        GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    time.sleep(SETTLE)
    down_vals = {p: GPIO.input(p) for p in ALL_GPIO}

    # 上拉观察
    for p in ALL_GPIO:
        GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    time.sleep(SETTLE)
    up_vals = {p: GPIO.input(p) for p in ALL_GPIO}

    for p in ALL_GPIO:
        # GPIO2/3 外部上拉特性，跳过“下拉仍高”的判定
        if p not in HW_PULLUP and down_vals[p] == GPIO.HIGH:
            issues.append(f"GPIO{p} (物理{BCM_TO_PHYSICAL[p]}) 下拉时仍为高，疑似粘连到 3V3")
        if up_vals[p] == GPIO.LOW:
            issues.append(f"GPIO{p} (物理{BCM_TO_PHYSICAL[p]}) 上拉时仍为低，疑似粘连到 GND")

    return issues


# -----------------------------
# 主程序
# -----------------------------
def main():
    ok, GPIO = diagnostics()
    if not ok or GPIO is None:
        print_header("结论")
        print_err("自诊断未通过，已停止粘连测试。请先修复环境问题后再运行。")
        sys.exit(2)

    print_header("3) 开始 GPIO 焊点粘连测试")
    print_warn("请确认：已断开所有外设/HAT/杜邦线。")
    print("测试中，请稍候...\n")

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    try:
        shorts = set()

        print("[1/3] 高电平轮询测试 ...")
        shorts |= test_drive(GPIO, GPIO.HIGH)

        print("[2/3] 低电平轮询测试 ...")
        shorts |= test_drive(GPIO, GPIO.LOW)

        print("[3/3] 供电/接地粘连粗检 ...")
        power_gnd_issues = test_stuck_to_power_or_gnd(GPIO)

        print_header("4) 测试结果")
        if not shorts:
            print_ok("未发现 GPIO 与 GPIO 之间的明显粘连")
        else:
            print_err(f"发现 {len(shorts)} 组疑似 GPIO-GPIO 粘连：")
            for a, b in sorted(shorts):
                print(f"  - GPIO{a} (物理{BCM_TO_PHYSICAL[a]}) <-> GPIO{b} (物理{BCM_TO_PHYSICAL[b]})")

        if not power_gnd_issues:
            print_ok("未发现明显 GPIO-3V3/GND 粘连迹象（粗检）")
        else:
            print_warn("发现疑似 GPIO-电源/地 粘连迹象：")
            for s in power_gnd_issues:
                print(f"  - {s}")

        print("\n建议：对所有疑似点位用万用表蜂鸣档复核，再进行返修。")

    except Exception as e:
        print_err(f"测试过程异常: {e}")
        msg = str(e).lower()
        if "xgpiohandlerequest" in msg or "invalid arguement" in msg or "invalid argument" in msg:
            print_err("再次出现 xGpioHandleRequest/Invalid argument：")
            print_warn("这通常是 GPIO 后端兼容问题，建议改用 libgpiod 版本脚本。")
        sys.exit(1)
    finally:
        try:
            GPIO.cleanup()
        except Exception:
            pass


if __name__ == "__main__":
    main()
