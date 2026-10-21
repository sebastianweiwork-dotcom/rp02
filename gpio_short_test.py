#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
树莓派 Zero 2 W GPIO 焊点粘连测试（含完整自诊断 + 异常引脚容错）

适配目标：
- Raspberry Pi OS / Debian on Pi
- RPi.GPIO 兼容层（含 rpi-lgpio）环境

功能：
1) 环境自诊断：
   - OS / Kernel / Python / 权限
   - /dev/gpiochip* 是否存在
   - raspi-gpio / pinctrl 工具可用性
   - RPi.GPIO 导入与版本
   - GPIO 最小读写自测

2) 粘连检测：
   - 逐个 GPIO 作为输出，其余 GPIO 作为输入，检测是否跟随电平变化
   - 高电平轮 + 低电平轮
   - 粗检疑似 GPIO->3V3 / GPIO->GND 粘连

3) 容错与可诊断：
   - 单个引脚申请失败不会中断全局测试
   - 记录“无法测试引脚”及错误原因
   - 可选输出 pinctrl 状态辅助排障

注意：
- 测试前请断开所有外设/HAT/杜邦线，避免误报。
- 本脚本主要检测 GPIO 间短路；电源脚短路请用万用表复核。
"""

import os
import sys
import time
import glob
import shutil
import platform
import subprocess
from typing import Dict, List, Set, Tuple, Optional

# -----------------------------
# 配置区
# -----------------------------
ALL_GPIO: List[int] = [
    2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
    17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27
]
HW_PULLUP = {2, 3}   # I2C1 SDA/SCL 板载硬件上拉
SETTLE = 0.008       # 电平稳定等待时间（秒）
SELFTEST_PIN = 17    # 最小读写自测引脚（可改）

BCM_TO_PHYSICAL: Dict[int, int] = {
    2: 3, 3: 5, 4: 7, 5: 29, 6: 31, 7: 26, 8: 24, 9: 21, 10: 19,
    11: 23, 12: 32, 13: 33, 14: 8, 15: 10, 16: 36, 17: 11, 18: 12,
    19: 35, 20: 38, 21: 40, 22: 15, 23: 16, 24: 18, 25: 22, 26: 37, 27: 13,
}

# 记录无法配置/读取的引脚，避免测试中断
FAILED_PINS: Dict[int, str] = {}


# -----------------------------
# 输出与系统工具
# -----------------------------
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


def run_cmd(cmd: List[str], timeout: int = 5) -> Tuple[int, str, str]:
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout.strip(), p.stderr.strip()
    except Exception as e:
        return 999, "", str(e)


def read_os_release() -> Dict[str, str]:
    info: Dict[str, str] = {}
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


# -----------------------------
# GPIO 安全操作封装
# -----------------------------
def active_pins() -> List[int]:
    return [p for p in ALL_GPIO if p not in FAILED_PINS]


def safe_cleanup_pin(GPIO, pin: int):
    try:
        GPIO.cleanup(pin)
    except Exception:
        pass


def safe_setup(GPIO, pin: int, direction, **kwargs) -> bool:
    """
    先 cleanup 再 setup，避免某些后端对重复 claim 报 Invalid argument。
    """
    if pin in FAILED_PINS:
        return False
    try:
        safe_cleanup_pin(GPIO, pin)
        GPIO.setup(pin, direction, **kwargs)
        return True
    except Exception as e:
        FAILED_PINS[pin] = f"{type(e).__name__}: {e}"
        print_warn(f"GPIO{pin} (物理{BCM_TO_PHYSICAL.get(pin, '?')}) 无法配置，已跳过: {e}")
        return False


def safe_read(GPIO, pin: int) -> Optional[int]:
    if pin in FAILED_PINS:
        return None
    try:
        return GPIO.input(pin)
    except Exception as e:
        FAILED_PINS[pin] = f"ReadError: {e}"
        print_warn(f"GPIO{pin} (物理{BCM_TO_PHYSICAL.get(pin, '?')}) 读取失败，已跳过: {e}")
        return None


def setup_all_input(GPIO, pull):
    for p in active_pins():
        safe_setup(GPIO, p, GPIO.IN, pull_up_down=pull)


# -----------------------------
# 自诊断
# -----------------------------
def diagnostics() -> Tuple[bool, object]:
    print_header("1) 环境自诊断")

    os_info = read_os_release()
    print_ok(f"OS: {os_info.get('PRETTY_NAME', 'Unknown OS')}")
    print_ok(f"Kernel: {platform.release()}")
    print_ok(f"Python: {sys.version.split()[0]}")

    if os.geteuid() == 0:
        print_ok("当前为 root 权限")
    else:
        print_warn("当前不是 root，建议用 sudo 运行（可能导致 GPIO 访问失败）")

    chips = sorted(glob.glob("/dev/gpiochip*"))
    if chips:
        print_ok(f"检测到 gpiochip 设备: {', '.join(chips)}")
    else:
        print_err("未发现 /dev/gpiochip*，GPIO 子系统可能不可用")

    if shutil.which("raspi-gpio"):
        rc, out, err = run_cmd(["raspi-gpio", "get", "17"], timeout=3)
        if rc == 0:
            print_ok("raspi-gpio 可用")
        else:
            print_warn(f"raspi-gpio 存在但调用非0: {err or out}")
    else:
        print_warn("raspi-gpio 不在 PATH 中（可选工具）")

    if shutil.which("pinctrl"):
        rc, out, err = run_cmd(["pinctrl", "get", "17"], timeout=3)
        if rc == 0:
            print_ok("pinctrl 可用")
        else:
            print_warn(f"pinctrl 存在但调用非0: {err or out}")
    else:
        print_warn("pinctrl 不在 PATH 中（可选工具）")

    # 导入 RPi.GPIO
    try:
        import RPi.GPIO as GPIO  # type: ignore
        ver = getattr(GPIO, "VERSION", "unknown")
        print_ok(f"RPi.GPIO 导入成功，版本: {ver}")
    except Exception as e:
        print_err(f"RPi.GPIO 导入失败: {e}")
        print_err("安装建议: sudo apt install python3-rpi.gpio")
        return False, None

    # 最小读写自测
    print_header("2) GPIO 最小读写自测")
    try:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        if not safe_setup(GPIO, SELFTEST_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN):
            raise RuntimeError(f"GPIO{SELFTEST_PIN} 输入配置失败")

        time.sleep(SETTLE)
        v = safe_read(GPIO, SELFTEST_PIN)
        if v is None:
            raise RuntimeError(f"GPIO{SELFTEST_PIN} 输入读取失败")
        print_ok(f"GPIO{SELFTEST_PIN} 输入下拉读取成功，值={v}")

        if not safe_setup(GPIO, SELFTEST_PIN, GPIO.OUT, initial=GPIO.LOW):
            raise RuntimeError(f"GPIO{SELFTEST_PIN} 输出配置失败")
        time.sleep(SETTLE)
        GPIO.output(SELFTEST_PIN, GPIO.HIGH)
        time.sleep(SETTLE)
        GPIO.output(SELFTEST_PIN, GPIO.LOW)
        time.sleep(SETTLE)
        print_ok(f"GPIO{SELFTEST_PIN} 输出高/低切换成功")

        safe_setup(GPIO, SELFTEST_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.cleanup()
        print_ok("GPIO cleanup 成功")

    except Exception as e:
        print_err(f"GPIO 最小自测失败: {e}")
        low = str(e).lower()
        if "xgpiohandlerequest" in low or "invalid argument" in low or "invalid arguement" in low:
            print_err("检测到 xGpioHandleRequest/Invalid argument，疑似 GPIO 后端兼容问题")
            print_warn("建议尝试：sudo 运行、减少 overlay 占用、改用 libgpiod 版本脚本")
        return False, None

    return True, GPIO


# -----------------------------
# 粘连检测
# -----------------------------
def test_drive(GPIO, level) -> Set[Tuple[int, int]]:
    """
    level=HIGH:
      - 其他引脚设为下拉输入
      - drv 输出高
      - 若其他引脚由低变高 -> 疑似与 drv 粘连
    level=LOW:
      - 其他引脚设为上拉输入
      - drv 输出低
      - 若其他引脚由高变低 -> 疑似与 drv 粘连
    """
    pull = GPIO.PUD_DOWN if level == GPIO.HIGH else GPIO.PUD_UP
    shorts: Set[Tuple[int, int]] = set()

    for drv in list(active_pins()):
        setup_all_input(GPIO, pull)
        time.sleep(SETTLE)

        idle = {p: safe_read(GPIO, p) for p in active_pins()}

        if not safe_setup(GPIO, drv, GPIO.OUT, initial=level):
            continue

        time.sleep(SETTLE)

        for p in active_pins():
            if p == drv:
                continue
            if level == GPIO.HIGH and p in HW_PULLUP:
                # GPIO2/3 硬件上拉，跳过“被拉高”判定
                continue

            now = safe_read(GPIO, p)
            base = idle.get(p)
            if now is None or base is None:
                continue

            if now == level and base != level:
                shorts.add(tuple(sorted((drv, p))))

        # 恢复 drv 为输入
        safe_setup(GPIO, drv, GPIO.IN, pull_up_down=pull)

    return shorts


def test_stuck_to_power_or_gnd(GPIO) -> List[str]:
    """
    粗检“单引脚被强拉”：
    - 下拉输入仍为高: 疑似粘 3V3（GPIO2/3 除外）
    - 上拉输入仍为低: 疑似粘 GND
    """
    issues: List[str] = []

    setup_all_input(GPIO, GPIO.PUD_DOWN)
    time.sleep(SETTLE)
    down_vals = {p: safe_read(GPIO, p) for p in active_pins()}

    setup_all_input(GPIO, GPIO.PUD_UP)
    time.sleep(SETTLE)
    up_vals = {p: safe_read(GPIO, p) for p in active_pins()}

    for p in active_pins():
        d = down_vals.get(p)
        u = up_vals.get(p)
        if d is None or u is None:
            continue

        if p not in HW_PULLUP and d == GPIO.HIGH:
            issues.append(f"GPIO{p} (物理{BCM_TO_PHYSICAL[p]}) 下拉时仍为高，疑似粘连到 3V3")
        if u == GPIO.LOW:
            issues.append(f"GPIO{p} (物理{BCM_TO_PHYSICAL[p]}) 上拉时仍为低，疑似粘连到 GND")

    return issues


def report_failed_pins():
    if not FAILED_PINS:
        return

    print_header("无法测试的引脚（已跳过）")
    for p, reason in sorted(FAILED_PINS.items()):
        phys = BCM_TO_PHYSICAL.get(p, "?")
        print_warn(f"GPIO{p} (物理{phys}): {reason}")
        if shutil.which("pinctrl"):
            rc, out, err = run_cmd(["pinctrl", "get", str(p)], timeout=3)
            if rc == 0:
                print(f"      pinctrl: {out}")
            else:
                print(f"      pinctrl: {err or out}")

    print_warn("说明：这些引脚可能被系统功能占用（I2C/SPI/UART/1-Wire/风扇等 overlay）")
    print_warn("建议临时禁用占用后复测，以获得完整覆盖。")


# -----------------------------
# 主程序
# -----------------------------
def main():
    ok, GPIO = diagnostics()
    if not ok or GPIO is None:
        print_header("结论")
        print_err("自诊断未通过，已停止粘连测试。请先修复环境问题后再试。")
        sys.exit(2)

    print_header("3) 开始 GPIO 焊点粘连测试")
    print_warn("请确认：已断开所有外设/HAT/杜邦线。")
    print("测试中，请稍候...\n")

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    try:
        shorts: Set[Tuple[int, int]] = set()

        print("[1/3] 高电平轮询测试 ...")
        shorts |= test_drive(GPIO, GPIO.HIGH)

        print("[2/3] 低电平轮询测试 ...")
        shorts |= test_drive(GPIO, GPIO.LOW)

        print("[3/3] 供电/接地粘连粗检 ...")
        power_gnd_issues = test_stuck_to_power_or_gnd(GPIO)

        report_failed_pins()

        print_header("4) 测试结果")
        if shorts:
            print_err(f"发现 {len(shorts)} 组疑似 GPIO-GPIO 粘连：")
            for a, b in sorted(shorts):
                print(f"  - GPIO{a} (物理{BCM_TO_PHYSICAL[a]}) <-> GPIO{b} (物理{BCM_TO_PHYSICAL[b]})")
        else:
            print_ok("未发现 GPIO 与 GPIO 之间的明显粘连（不含被跳过引脚）")

        if power_gnd_issues:
            print_warn("发现疑似 GPIO-3V3/GND 粘连迹象：")
            for s in power_gnd_issues:
                print(f"  - {s}")
        else:
            print_ok("未发现明显 GPIO-3V3/GND 粘连迹象（粗检）")

        if FAILED_PINS:
            print_warn("存在被跳过引脚；当前结论不覆盖这些引脚。建议排除占用后复测。")

        print("\n建议：对所有疑似点位使用万用表蜂鸣档复核，再进行返修。")

    except KeyboardInterrupt:
        print_warn("用户中断测试。")
        sys.exit(130)
    except Exception as e:
        print_err(f"测试过程异常: {e}")
        low = str(e).lower()
        if "xgpiohandlerequest" in low or "invalid argument" in low or "invalid arguement" in low:
            print_err("检测到 xGpioHandleRequest/Invalid argument")
            print_warn("建议改用 libgpiod 版本脚本（在 Debian Trixie/Bookworm 上更稳）")
        sys.exit(1)
    finally:
        try:
            GPIO.cleanup()
        except Exception:
            pass


if __name__ == "__main__":
    main()
