#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
树莓派 Zero 2 W 红外对射测试
接线(你提供的)：
- 发射器: VCC -> Pin1(3.3V), GND -> Pin30(GND)
- 接收器: VCC -> Pin17(3.3V), GND -> Pin9(GND), OUT -> Pin11(GPIO17)

功能：
1) 实时显示状态（遮挡/未遮挡）
2) 统计状态变化次数
3) 可选超时自动退出
"""

import time
import argparse
import RPi.GPIO as GPIO

PIN = 17  # BCM 编号，物理11

def read_state(pin: int) -> bool:
    """
    返回 True 表示“光路畅通”(未遮挡)，False 表示“被遮挡”
    注意：不同模块电平逻辑可能相反，若反了把判断改一下即可
    """
    v = GPIO.input(pin)
    # 常见对射接收模块：收到红外时 OUT=LOW；遮挡时 OUT=HIGH
    # 如果你模块相反，把下一行改成: return (v == GPIO.HIGH)
    return (v == GPIO.LOW)

def main():
    parser = argparse.ArgumentParser(description="IR break-beam test on Raspberry Pi GPIO17")
    parser.add_argument("--seconds", type=int, default=0, help="测试时长(秒), 0=一直运行")
    parser.add_argument("--pull", choices=["up", "down"], default="up",
                        help="输入上下拉，默认 up（多数模块更稳）")
    args = parser.parse_args()

    pull_mode = GPIO.PUD_UP if args.pull == "up" else GPIO.PUD_DOWN

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN, GPIO.IN, pull_up_down=pull_mode)

    print("=== 红外对射测试开始 ===")
    print("GPIO17 (物理11) 读取中...")
    print("请在发射器与接收器之间来回遮挡，观察状态变化。")
    if args.seconds > 0:
        print(f"测试时长: {args.seconds} 秒")
    print("按 Ctrl+C 可退出。\n")

    start = time.time()
    last = read_state(PIN)
    changes = 0

    print(f"[INIT] 状态: {'未遮挡(光路通)' if last else '遮挡(光路断)'}")

    try:
        while True:
            cur = read_state(PIN)
            if cur != last:
                changes += 1
                ts = time.strftime("%H:%M:%S")
                print(f"[{ts}] 状态变化 -> {'未遮挡(光路通)' if cur else '遮挡(光路断)'}  (变化次数={changes})")
                last = cur

            if args.seconds > 0 and (time.time() - start) >= args.seconds:
                break

            time.sleep(0.01)  # 10ms 轮询

    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()

    print("\n=== 测试结束 ===")
    print(f"总变化次数: {changes}")
    if changes >= 2:
        print("✅ 模块基本工作正常（能检测到遮挡变化）")
    else:
        print("⚠️ 未观察到足够状态变化，请检查对准、供电、接线和电平逻辑。")

if __name__ == "__main__":
    main()
