#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
红外对射触发拍照 + 性能记录 (Pi Zero 2 W, GPIO17, rpicam-still)

每次对射被遮挡 -> 拍一张 JPG -> 记录:
  - 各阶段延迟: 触发检测(消抖)、拍照启动、拍照总耗时、保存/落盘、总延迟
  - CPU 整体占用、进程内存、系统内存、温度、CPU频率、降频/欠压标志(vcgencmd)
  - 文件大小、磁盘剩余空间、遮挡持续时间
结果写入 CSV: <outdir>/log.csv

用法:
  sudo python3 ir_trigger_camera.py --blocked-level 0
  参数 --blocked-level: 遮挡时的电平 (0 或 1)。可先用自校准脚本确认。
"""

import os
import sys
import csv
import time
import shutil
import argparse
import subprocess
from datetime import datetime

import RPi.GPIO as GPIO

PIN = 17


# ---------------- 系统信息 ----------------
def read_cpu_times():
    with open("/proc/stat") as f:
        parts = f.readline().split()[1:]
    vals = list(map(int, parts))
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
    return idle, sum(vals)


def cpu_percent(prev):
    idle, total = read_cpu_times()
    pidle, ptotal = prev
    dt = total - ptotal
    pct = 100.0 * (1 - (idle - pidle) / dt) if dt > 0 else 0.0
    return pct, (idle, total)


def mem_info():
    d = {}
    with open("/proc/meminfo") as f:
        for line in f:
            k, v = line.split(":")
            d[k] = int(v.split()[0])  # kB
    total = d["MemTotal"] / 1024
    avail = d["MemAvailable"] / 1024
    return total, avail, 100.0 * (total - avail) / total


def proc_rss_mb():
    with open("/proc/self/status") as f:
        for line in f:
            if line.startswith("VmRSS"):
                return int(line.split()[1]) / 1024
    return 0.0


def cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read()) / 1000.0
    except Exception:
        return float("nan")


def cpu_freq_mhz():
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq") as f:
            return int(f.read()) / 1000.0
    except Exception:
        return float("nan")


def throttled():
    """vcgencmd get_throttled -> 十六进制标志, 0x0 表示正常"""
    if not shutil.which("vcgencmd"):
        return "n/a"
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                             text=True, timeout=2).stdout.strip()
        return out.split("=")[-1]
    except Exception:
        return "n/a"


def load1():
    return os.getloadavg()[0]


# ---------------- 主逻辑 ----------------
FIELDS = [
    "seq", "time", "file",
    "debounce_ms", "spawn_to_done_ms", "capture_total_ms", "stat_ms", "trigger_to_file_ms",
    "block_duration_ms", "missed_triggers_during_capture",
    "file_kb", "cpu_pct_during", "load1", "proc_rss_mb",
    "mem_used_pct", "mem_avail_mb", "temp_c_before", "temp_c_after",
    "cpu_mhz", "throttled", "disk_free_mb", "rc",
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--blocked-level", type=int, choices=[0, 1], default=0,
                    help="遮挡时 OUT 的电平 (默认0)")
    ap.add_argument("--pull", choices=["up", "down", "off"], default="up")
    ap.add_argument("--outdir", default=os.path.expanduser("~/ir_shots"))
    ap.add_argument("--debounce", type=float, default=0.02, help="消抖时间(秒)")
    ap.add_argument("--cooldown", type=float, default=0.0, help="拍照后最小冷却(秒)")
    ap.add_argument("--timeout-ms", type=int, default=1,
                    help="rpicam-still 预热时间(ms)。越小越快但曝光/白平衡可能不稳")
    ap.add_argument("--width", type=int, default=2592)
    ap.add_argument("--height", type=int, default=1944)
    ap.add_argument("--quality", type=int, default=90)
    ap.add_argument("--max", type=int, default=0, help="最多拍几张, 0=不限")
    args = ap.parse_args()

    if not shutil.which("rpicam-still"):
        sys.exit("未找到 rpicam-still")

    os.makedirs(args.outdir, exist_ok=True)
    csv_path = os.path.join(args.outdir, "log.csv")
    new_file = not os.path.exists(csv_path)
    csv_f = open(csv_path, "a", newline="", encoding="utf-8")
    w = csv.DictWriter(csv_f, fieldnames=FIELDS)
    if new_file:
        w.writeheader()

    pud = {"up": GPIO.PUD_UP, "down": GPIO.PUD_DOWN, "off": GPIO.PUD_OFF}[args.pull]
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN, GPIO.IN, pull_up_down=pud)

    blocked = args.blocked_level
    clear = 1 - blocked
    print(f"遮挡电平={blocked}, 输出目录={args.outdir}, 日志={csv_path}")
    print("等待遮挡触发, Ctrl+C 退出\n")

    seq = 0
    cpu_prev = read_cpu_times()
    last_shot_end = 0.0

    try:
        # 等待初始状态为未遮挡, 避免启动即触发
        while GPIO.input(PIN) == blocked:
            time.sleep(0.01)

        while True:
            # 1) 等待下降沿(进入遮挡)
            if GPIO.input(PIN) != blocked:
                time.sleep(0.002)
                continue

            t_detect = time.monotonic()

            # 2) 消抖: 持续遮挡 debounce 秒才算有效
            ok = True
            t_end = t_detect + args.debounce
            while time.monotonic() < t_end:
                if GPIO.input(PIN) != blocked:
                    ok = False
                    break
                time.sleep(0.001)
            if not ok:
                continue
            if time.monotonic() - last_shot_end < args.cooldown:
                while GPIO.input(PIN) == blocked:
                    time.sleep(0.01)
                continue

            t_debounced = time.monotonic()
            seq += 1
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            fpath = os.path.join(args.outdir, f"shot_{seq:05d}_{ts}.jpg")

            # 拍照前状态
            _, cpu_prev = cpu_percent(cpu_prev)  # 重置采样基线
            temp_before = cpu_temp()

            # 3) 拍照
            cmd = ["rpicam-still", "-n", "-o", fpath,
                   "-t", str(args.timeout_ms),
                   "--width", str(args.width), "--height", str(args.height),
                   "-q", str(args.quality)]
            t_spawn = time.monotonic()
            proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            # 拍照期间记录是否有新的遮挡/恢复(用于统计漏触发)
            missed = 0
            prev_lvl = GPIO.input(PIN)
            while proc.poll() is None:
                lvl = GPIO.input(PIN)
                if prev_lvl == clear and lvl == blocked:
                    missed += 1
                prev_lvl = lvl
                time.sleep(0.002)
            rc = proc.returncode
            t_done = time.monotonic()

            cpu_pct, cpu_prev = cpu_percent(cpu_prev)
            temp_after = cpu_temp()

            # 4) 落盘检查
            t_s0 = time.monotonic()
            try:
                size_kb = os.path.getsize(fpath) / 1024
            except OSError:
                size_kb = 0.0
            t_s1 = time.monotonic()

            # 等待遮挡结束, 统计遮挡持续时间 (一次遮挡只拍一张)
            while GPIO.input(PIN) == blocked:
                time.sleep(0.005)
            t_release = time.monotonic()
            last_shot_end = t_release

            _, mem_avail, mem_pct = mem_info()
            row = {
                "seq": seq,
                "time": datetime.now().isoformat(timespec="milliseconds"),
                "file": os.path.basename(fpath),
                "debounce_ms": round((t_debounced - t_detect) * 1000, 1),
                "spawn_to_done_ms": round((t_done - t_spawn) * 1000, 1),
                "capture_total_ms": round((t_done - t_debounced) * 1000, 1),
                "stat_ms": round((t_s1 - t_s0) * 1000, 2),
                "trigger_to_file_ms": round((t_s1 - t_detect) * 1000, 1),
                "block_duration_ms": round((t_release - t_detect) * 1000, 1),
                "missed_triggers_during_capture": missed,
                "file_kb": round(size_kb, 1),
                "cpu_pct_during": round(cpu_pct, 1),
                "load1": round(load1(), 2),
                "proc_rss_mb": round(proc_rss_mb(), 1),
                "mem_used_pct": round(mem_pct, 1),
                "mem_avail_mb": round(mem_avail, 1),
                "temp_c_before": round(temp_before, 1),
                "temp_c_after": round(temp_after, 1),
                "cpu_mhz": round(cpu_freq_mhz(), 0),
                "throttled": throttled(),
                "disk_free_mb": round(shutil.disk_usage(args.outdir).free / 1048576, 0),
                "rc": rc,
            }
            w.writerow(row)
            csv_f.flush()

            flag = "OK " if rc == 0 and size_kb > 0 else "ERR"
            print(f"[{flag}] #{seq} {row['file']} | 触发->文件 {row['trigger_to_file_ms']}ms "
                  f"(拍照 {row['spawn_to_done_ms']}ms) | {row['file_kb']}KB | "
                  f"CPU {row['cpu_pct_during']}% | 内存 {row['mem_used_pct']}% | "
                  f"{row['temp_c_before']}→{row['temp_c_after']}°C | throttled={row['throttled']}"
                  + (f" | 拍照期间漏触发 {missed}" if missed else ""))

            if args.max and seq >= args.max:
                break

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        GPIO.cleanup()
        csv_f.close()
        print(f"共拍摄 {seq} 张, 日志: {csv_path}")


if __name__ == "__main__":
    main()
