#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
红外对射(发射+接收)自校准测试脚本 - Raspberry Pi GPIO17

接线:
- 发射器: VCC -> Pin1(3.3V), GND -> Pin30(GND)
- 接收器: VCC -> Pin17(3.3V), GND -> Pin9(GND), OUT -> Pin11(GPIO17)

功能:
1) 启动后先记录当前电平
2) 引导你在校准窗口内“遮挡一次并移开”
3) 自动判断哪种电平代表“未遮挡/遮挡”
4) 实时输出状态变化，统计触发次数
"""

import time
import argparse
import RPi.GPIO as GPIO

PIN = 17  # BCM


def calibrate(pin: int, seconds: float = 6.0, sample_interval: float = 0.002):
    """
    在校准窗口内采样，要求用户做“遮挡->移开”动作。
    返回:
      clear_level: 未遮挡时电平(0/1)
      blocked_level: 遮挡时电平(0/1)
    """
    print("\n=== 自校准开始 ===")
    print("请确保现在是【未遮挡】状态，并在接下来几秒内执行：")
    print("  1) 用手遮挡光路 1~2 秒")
    print("  2) 再移开恢复未遮挡")
    print(f"校准采样时长: {seconds:.1f}s\n")

    t_end = time.time() + seconds
    samples = []
    while time.time() < t_end:
        samples.append(GPIO.input(pin))
        time.sleep(sample_interval)

    if len(samples) < 10:
        raise RuntimeError("采样数量过少，校准失败")

    uniq = sorted(set(samples))
    if len(uniq) == 1:
        lvl = uniq[0]
        print("⚠️ 校准期间电平没有变化，无法自动判断极性。")
        print("将采用默认假设: 未遮挡=HIGH, 遮挡=LOW")
        return 1, 0

    # 找出最长连续段，通常对应“未遮挡”的稳定状态（你开始和结束都在未遮挡）
    runs = []
    cur = samples[0]
    run_len = 1
    for v in samples[1:]:
        if v == cur:
            run_len += 1
        else:
            runs.append((cur, run_len))
            cur = v
            run_len = 1
    runs.append((cur, run_len))

    total_by_level = {0: 0, 1: 0}
    for lv, ln in runs:
        total_by_level[lv] += ln

    clear_level = 1 if total_by_level[1] >= total_by_level[0] else 0
    blocked_level = 0 if clear_level == 1 else 1

    print("✅ 校准完成:")
    print(f"  未遮挡(光路通) => 电平 {'HIGH' if clear_level else 'LOW'}")
    print(f"  遮挡(光路断)   => 电平 {'HIGH' if blocked_level else 'LOW'}\n")
    return clear_level, blocked_level


def state_text(level: int, clear_level: int) -> str:
    return "未遮挡(光路通)" if level == clear_level else "遮挡(光路断)"


def main():
    parser = argparse.ArgumentParser(description="IR break-beam self-calibration tester")
    parser.add_argument("--seconds", type=int, default=0, help="测试时长(秒)，0=持续运行")
    parser.add_argument("--pull", choices=["up", "down", "off"], default="up",
                        help="GPIO内部上下拉，默认up")
    parser.add_argument("--calib", type=float, default=6.0, help="校准时长(秒)，默认6")
    args = parser.parse_args()

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    if args.pull == "up":
        pud = GPIO.PUD_UP
    elif args.pull == "down":
        pud = GPIO.PUD_DOWN
    else:
        pud = GPIO.PUD_OFF

    GPIO.setup(PIN, GPIO.IN, pull_up_down=pud)

    print("=== 红外对射测试(自校准版) ===")
    print("GPIO: BCM17 / 物理Pin11")
    print(f"内部上下拉: {args.pull}")
    print("按 Ctrl+C 可退出。")

    try:
        clear_level, blocked_level = calibrate(PIN, seconds=args.calib)

        print("=== 进入实时监测 ===")
        start = time.time()
        last = GPIO.input(PIN)
        changes = 0
        blocked_count = 0
        clear_count = 0

        print(f"[INIT] 当前状态: {state_text(last, clear_level)}")

        while True:
            cur = GPIO.input(PIN)
            if cur != last:
                changes += 1
                st = state_text(cur, clear_level)
                if cur == clear_level:
                    clear_count += 1
                else:
                    blocked_count += 1
                ts = time.strftime("%H:%M:%S")
                print(f"[{ts}] 状态变化 -> {st} (总变化={changes})")
                last = cur

            if args.seconds > 0 and (time.time() - start) >= args.seconds:
                break

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n用户中断。")
    finally:
        GPIO.cleanup()

    print("\n=== 测试结束 ===")
    print(f"状态变化次数: {changes}")
    print(f"进入“遮挡”次数: {blocked_count}")
    print(f"进入“未遮挡”次数: {clear_count}")

    if changes >= 2 and blocked_count >= 1 and clear_count >= 1:
        print("✅ 对射模块工作正常（可检测遮挡与恢复）")
    else:
        print("⚠️ 未观察到完整遮挡/恢复过程，请检查:")
        print("   1) 发射与接收是否对准")
        print("   2) 接线与供电是否稳定")
        print("   3) 环境强光干扰")
        print("   4) 适当增大 --calib 时间后重试")


if __name__ == "__main__":
    main()
