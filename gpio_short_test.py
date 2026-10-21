#!/usr/bin/env python3
"""
树莓派 Zero 2 W GPIO 焊点粘连(短路)测试
原理:逐个引脚作为输出,其余引脚作为输入,检测是否有其他引脚跟随电平变化。
依赖:sudo apt install python3-rpi.gpio   (或 pip install RPi.GPIO)
运行:python3 gpio_short_test.py
警告:运行前请断开所有外接设备/HAT/杜邦线!
"""
import time
import RPi.GPIO as GPIO

# 40pin 排针上全部可用的 GPIO (BCM 编号), 共 26 个
# 注意: GPIO2/3 (I2C) 板上带有 1.8k 外部上拉电阻, 下拉测试时会被拉高, 需要特殊处理
ALL_GPIO = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
            17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27]

# 有硬件外部上拉的引脚(I2C1 SDA/SCL), 单独处理
HW_PULLUP = {2, 3}

# BCM -> 物理引脚号 (便于定位焊点)
BCM_TO_PHYSICAL = {
    2: 3, 3: 5, 4: 7, 5: 29, 6: 31, 7: 26, 8: 24, 9: 21, 10: 19,
    11: 23, 12: 32, 13: 33, 14: 8, 15: 10, 16: 36, 17: 11, 18: 12,
    19: 35, 20: 38, 21: 40, 22: 15, 23: 16, 24: 18, 25: 22, 26: 37, 27: 13,
}

SETTLE = 0.005  # 电平稳定时间(秒)


def setup_all_input(pull):
    for p in ALL_GPIO:
        GPIO.setup(p, GPIO.IN, pull_up_down=pull)


def test_drive(level):
    """
    level=HIGH: 其余引脚下拉, 驱动一个引脚为高, 检查其他引脚是否变高
    level=LOW : 其余引脚上拉, 驱动一个引脚为低, 检查其他引脚是否变低
    """
    pull = GPIO.PUD_DOWN if level == GPIO.HIGH else GPIO.PUD_UP
    shorts = set()

    for drv in ALL_GPIO:
        setup_all_input(pull)
        time.sleep(SETTLE)

        # 基线: 先确认没有驱动时, 各引脚处于预期的空闲电平
        idle = {p: GPIO.input(p) for p in ALL_GPIO}

        GPIO.setup(drv, GPIO.OUT, initial=level)
        time.sleep(SETTLE)

        for p in ALL_GPIO:
            if p == drv:
                continue
            # 跳过硬件上拉引脚在"高电平测试"中的判断(本来就是高)
            if level == GPIO.HIGH and p in HW_PULLUP:
                continue
            val = GPIO.input(p)
            if val == level and idle[p] != level:
                shorts.add(tuple(sorted((drv, p))))

        # 恢复为输入, 避免长时间驱动
        GPIO.setup(drv, GPIO.IN, pull_up_down=pull)

    return shorts


def test_hw_pullup_pins():
    """GPIO2/3 有外部上拉: 驱动为低, 检查其余引脚(上拉)是否被拉低; 已在 LOW 测试中覆盖。"""
    pass


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    print("=== 树莓派 Zero 2 W GPIO 粘连测试 ===")
    print("请确认已断开所有外接设备!\n")

    try:
        shorts = set()
        print("[1/2] 驱动高电平测试 ...")
        shorts |= test_drive(GPIO.HIGH)
        print("[2/2] 驱动低电平测试 ...")
        shorts |= test_drive(GPIO.LOW)

        print("\n=== 测试结果 ===")
        if not shorts:
            print("✅ 未发现 GPIO 之间粘连。")
        else:
            print(f"❌ 发现 {len(shorts)} 组疑似粘连:")
            for a, b in sorted(shorts):
                print(f"  GPIO{a} (物理引脚 {BCM_TO_PHYSICAL[a]})  <->  "
                      f"GPIO{b} (物理引脚 {BCM_TO_PHYSICAL[b]})")
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
