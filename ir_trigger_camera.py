#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
红外对射触发拍照（Picamera2 常驻低延迟版）
------------------------------------------------
特点：
- 相机常驻运行，不再每次启动 rpicam-still（显著降低触发延迟）
- 每次遮挡触发抓拍 1 张 JPG
- 记录完整日志：延迟、CPU、内存、温度、频率、throttled、文件大小、磁盘空间等
- 结果输出到 CSV

硬件：
- IR 接收器 OUT -> GPIO17 (物理 11)

运行：
  sudo python3 ir_trigger_camera.py
  （默认 --blocked-level 0）
"""

import os
import csv
import time
import argparse
import shutil
import subprocess
from datetime import datetime

import RPi.GPIO as GPIO
from picamera2 import Picamera2
from PIL import Image
import io

PIN = 17

FIELDS = [
    "seq", "time", "file",
    "debounce_ms", "capture_call_ms", "jpeg_encode_ms", "write_file_ms",
    "trigger_to_file_ms", "block_duration_ms",
    "missed_triggers_during_capture",
    "width", "height", "jpeg_quality", "file_kb",
    "cpu_pct_during", "load1", "proc_rss_mb",
    "mem_used_pct", "mem_avail_mb",
    "temp_c_before", "temp_c_after",
    "cpu_mhz", "throttled", "disk_free_mb"
]


def read_cpu_times():
    with open("/proc/stat", "r", encoding="utf-8") as f:
        parts = f.readline().split()[1:]
    vals = list(map(int, parts))
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
    total = sum(vals)
    return idle, total


def cpu_percent(prev):
    idle, total = read_cpu_times()
    p_idle, p_total = prev
    dt = total - p_total
    pct = 100.0 * (1 - (idle - p_idle) / dt) if dt > 0 else 0.0
    return pct, (idle, total)


def mem_info():
    d = {}
    with open("/proc/meminfo", "r", encoding="utf-8") as f:
        for line in f:
            k, v = line.split(":", 1)
            d[k] = int(v.strip().split()[0])  # kB
    total_mb = d.get("MemTotal", 0) / 1024
    avail_mb = d.get("MemAvailable", 0) / 1024
    used_pct = (100.0 * (total_mb - avail_mb) / total_mb) if total_mb else 0.0
    return total_mb, avail_mb, used_pct


def proc_rss_mb():
    with open("/proc/self/status", "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024
    return 0.0


def cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r", encoding="utf-8") as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return float("nan")


def cpu_mhz():
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", "r", encoding="utf-8") as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return float("nan")


def throttled():
    if not shutil.which("vcgencmd"):
        return "n/a"
    try:
        out = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True, text=True, timeout=2
        ).stdout.strip()
        return out.split("=")[-1] if "=" in out else out
    except Exception:
        return "n/a"


def load1():
    return os.getloadavg()[0]


def main():
    ap = argparse.ArgumentParser()
    # 改为可选参数，默认 0
    ap.add_argument("--blocked-level", type=int, choices=[0, 1], default=0,
                    help="遮挡时 GPIO 电平: 0 或 1（默认 0）")
    ap.add_argument("--pull", choices=["up", "down", "off"], default="up",
                    help="GPIO 内部上下拉")
    ap.add_argument("--debounce-ms", type=float, default=20.0,
                    help="触发消抖时长(毫秒)")
    ap.add_argument("--cooldown-ms", type=float, default=0.0,
                    help="一次拍照后冷却时间(毫秒)")
    ap.add_argument("--outdir", default=os.path.expanduser("~/ir_shots_picam2"),
                    help="图片与日志输出目录")
    ap.add_argument("--max-shots", type=int, default=0,
                    help="最多拍摄数量，0=不限")
    ap.add_argument("--width", type=int, default=1296,
                    help="输出宽度，默认 1296（更低延迟）")
    ap.add_argument("--height", type=int, default=972,
                    help="输出高度，默认 972（更低延迟）")
    ap.add_argument("--jpeg-quality", type=int, default=90)
    ap.add_argument("--af", action="store_true", help="启用连续自动对焦（若镜头支持）")
    args = ap.parse_args()

    blocked = args.blocked_level
    clear = 1 - blocked
    debounce_s = args.debounce_ms / 1000.0
    cooldown_s = args.cooldown_ms / 1000.0

    os.makedirs(args.outdir, exist_ok=True)
    csv_path = os.path.join(args.outdir, "log.csv")
    create_header = not os.path.exists(csv_path)

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    pud = {"up": GPIO.PUD_UP, "down": GPIO.PUD_DOWN, "off": GPIO.PUD_OFF}[args.pull]
    GPIO.setup(PIN, GPIO.IN, pull_up_down=pud)

    picam2 = Picamera2()
    config = picam2.create_still_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"},
        buffer_count=3
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.3)

    if args.af:
        try:
            picam2.set_controls({"AfMode": 2})
        except Exception:
            pass

    print("=== Picamera2 常驻触发拍照 ===")
    print(f"blocked_level={blocked}, GPIO{PIN}, outdir={args.outdir}")
    print(f"size={args.width}x{args.height}, jpeg_quality={args.jpeg_quality}")
    print("等待遮挡触发，Ctrl+C 退出...\n")

    f = open(csv_path, "a", newline="", encoding="utf-8")
    writer = csv.DictWriter(f, fieldnames=FIELDS)
    if create_header:
        writer.writeheader()

    seq = 0
    cpu_prev = read_cpu_times()
    last_end = 0.0

    try:
        while GPIO.input(PIN) == blocked:
            time.sleep(0.005)

        while True:
            if GPIO.input(PIN) != blocked:
                time.sleep(0.001)
                continue

            t_detect = time.monotonic()

            stable = True
            t_db_end = t_detect + debounce_s
            while time.monotonic() < t_db_end:
                if GPIO.input(PIN) != blocked:
                    stable = False
                    break
                time.sleep(0.0008)
            if not stable:
                continue

            if time.monotonic() - last_end < cooldown_s:
                while GPIO.input(PIN) == blocked:
                    time.sleep(0.001)
                continue

            t_debounced = time.monotonic()
            seq += 1
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = f"shot_{seq:05d}_{ts}.jpg"
            filepath = os.path.join(args.outdir, filename)

            _, cpu_prev = cpu_percent(cpu_prev)
            temp_before = cpu_temp()

            t0 = time.monotonic()
            frame = picam2.capture_array("main")
            t1 = time.monotonic()

            t2 = time.monotonic()
            img = Image.fromarray(frame)
            bio = io.BytesIO()
            img.save(bio, format="JPEG", quality=args.jpeg_quality, optimize=False)
            jpeg_bytes = bio.getvalue()
            t3 = time.monotonic()

            t4 = time.monotonic()
            with open(filepath, "wb") as wf:
                wf.write(jpeg_bytes)
                wf.flush()
                os.fsync(wf.fileno())
            t5 = time.monotonic()

            missed = 0
            prev = GPIO.input(PIN)
            obs_end = time.monotonic() + 0.03
            while time.monotonic() < obs_end:
                cur = GPIO.input(PIN)
                if prev == clear and cur == blocked:
                    missed += 1
                prev = cur
                time.sleep(0.001)

            while GPIO.input(PIN) == blocked:
                time.sleep(0.001)
            t_release = time.monotonic()
            last_end = t_release

            temp_after = cpu_temp()
            cpu_pct, cpu_prev = cpu_percent(cpu_prev)
            _, mem_avail, mem_used_pct = mem_info()
            size_kb = os.path.getsize(filepath) / 1024.0

            row = {
                "seq": seq,
                "time": datetime.now().isoformat(timespec="milliseconds"),
                "file": filename,
                "debounce_ms": round((t_debounced - t_detect) * 1000, 2),
                "capture_call_ms": round((t1 - t0) * 1000, 2),
                "jpeg_encode_ms": round((t3 - t2) * 1000, 2),
                "write_file_ms": round((t5 - t4) * 1000, 2),
                "trigger_to_file_ms": round((t5 - t_detect) * 1000, 2),
                "block_duration_ms": round((t_release - t_detect) * 1000, 2),
                "missed_triggers_during_capture": missed,
                "width": args.width,
                "height": args.height,
                "jpeg_quality": args.jpeg_quality,
                "file_kb": round(size_kb, 1),
                "cpu_pct_during": round(cpu_pct, 1),
                "load1": round(load1(), 2),
                "proc_rss_mb": round(proc_rss_mb(), 1),
                "mem_used_pct": round(mem_used_pct, 1),
                "mem_avail_mb": round(mem_avail, 1),
                "temp_c_before": round(temp_before, 1),
                "temp_c_after": round(temp_after, 1),
                "cpu_mhz": round(cpu_mhz(), 0),
                "throttled": throttled(),
                "disk_free_mb": round(shutil.disk_usage(args.outdir).free / (1024 * 1024), 0),
            }

            writer.writerow(row)
            f.flush()

            print(
                f"[OK] #{seq} {filename} | 触发->文件 {row['trigger_to_file_ms']}ms "
                f"(抓帧 {row['capture_call_ms']} + 编码 {row['jpeg_encode_ms']} + 写盘 {row['write_file_ms']}) "
                f"| {row['file_kb']}KB | CPU {row['cpu_pct_during']}% | MEM {row['mem_used_pct']}% "
                f"| {row['temp_c_before']}→{row['temp_c_after']}°C | throttled={row['throttled']}"
            )

            if args.max_shots and seq >= args.max_shots:
                break

    except KeyboardInterrupt:
        print("\n用户中断。")
    finally:
        try:
            picam2.stop()
        except Exception:
            pass
        GPIO.cleanup()
        f.close()
        print(f"\n结束。共拍摄 {seq} 张。日志: {csv_path}")


if __name__ == "__main__":
    main()
