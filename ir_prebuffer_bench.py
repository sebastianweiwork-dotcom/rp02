#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
预缓冲时间对齐基准测试 (Pi Zero 2 W + OV5647 + 红外对射 GPIO17)

目标：
- 优先优化“画面时间尽可能接近触发瞬间”
- 触发后不直接取 latest()，而是等待一小段时间，再从缓冲里按时间戳选最接近触发时刻的帧
- 每次触发保存三张: prev / closest / next（围绕 closest）

矩阵:
  分辨率: 640x480, 1296x972, 1920x1080
  JPEG质量: 70, 90
  输出介质: SD卡, tmpfs
  每组触发: 3 次
总计: 12组 * 3 = 36 次触发（每次保存3张图）

运行:
  sudo python3 ir_prebuffer_bench.py --blocked-level 0
"""

import os
import io
import csv
import time
import shutil
import argparse
import threading
import subprocess
import statistics
from collections import deque
from datetime import datetime

import RPi.GPIO as GPIO
from picamera2 import Picamera2
from PIL import Image

PIN = 17
RESOLUTIONS = [(640, 480), (1296, 972), (1920, 1080)]
QUALITIES = [70, 90]
STORAGES = [("sd", os.path.expanduser("~/ir_bench")), ("tmpfs", "/dev/shm/ir_bench")]
SHOTS_PER_CONFIG = 3


def now_ns():
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)


def cpu_times():
    with open("/proc/stat", "r", encoding="utf-8") as f:
        vals = list(map(int, f.readline().split()[1:]))
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
    total = sum(vals)
    return idle, total


def cpu_pct(prev):
    idle, total = cpu_times()
    dt = total - prev[1]
    pct = 100.0 * (1 - (idle - prev[0]) / dt) if dt > 0 else 0.0
    return pct, (idle, total)


def mem_used_pct():
    d = {}
    with open("/proc/meminfo", "r", encoding="utf-8") as f:
        for line in f:
            k, v = line.split(":", 1)
            d[k] = int(v.strip().split()[0])
    return 100.0 * (d["MemTotal"] - d["MemAvailable"]) / d["MemTotal"]


def temp_c():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r", encoding="utf-8") as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return float("nan")


def throttled():
    try:
        out = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True, text=True, timeout=2
        ).stdout.strip()
        return out.split("=")[-1] if "=" in out else out
    except Exception:
        return "n/a"


def ring_size_for_resolution(w, h):
    # 避免 512MB 内存被大分辨率环形缓冲撑爆
    if (w, h) == (640, 480):
        return 32
    if (w, h) == (1296, 972):
        return 12
    return 8  # 1920x1080


def wait_ms_for_resolution(w, h):
    # 触发后等待 1~2 帧，让“触发后帧”进入缓冲，提升最近匹配概率
    # 640x480 标称高帧率，等待可短；高分辨率可稍长
    if (w, h) == (640, 480):
        return 35.0
    if (w, h) == (1296, 972):
        return 45.0
    return 55.0  # 1920x1080


class FrameRing:
    """
    后台持续采帧到环形缓冲:
      每项: (sensor_ts_ns, host_capture_ns, frame_array)
    """
    def __init__(self, picam2, maxlen):
        self.cam = picam2
        self.buf = deque(maxlen=maxlen)
        self.lock = threading.Lock()
        self.running = False
        self.th = None
        self.fps = 0.0

    def start(self):
        self.running = True
        self.th = threading.Thread(target=self._loop, daemon=True)
        self.th.start()

    def stop(self):
        self.running = False
        if self.th:
            self.th.join(timeout=3)
        with self.lock:
            self.buf.clear()

    def _loop(self):
        last = time.monotonic()
        while self.running:
            try:
                req = self.cam.capture_request()
            except Exception:
                break
            host_ns = now_ns()
            try:
                arr = req.make_array("main")
                md = req.get_metadata() or {}
                sensor_ts = md.get("SensorTimestamp", host_ns)
            finally:
                req.release()

            with self.lock:
                self.buf.append((int(sensor_ts), int(host_ns), arr))

            cur = time.monotonic()
            inst = 1.0 / max(cur - last, 1e-6)
            self.fps = 0.9 * self.fps + 0.1 * inst if self.fps > 0 else inst
            last = cur

    def snapshot(self):
        with self.lock:
            return list(self.buf)


def pick_closest_index(frames, trigger_ns):
    # 按 |sensor_ts - trigger| 最小选帧
    best_i = None
    best_d = None
    for i, (sensor_ts, _, _) in enumerate(frames):
        d = abs(sensor_ts - trigger_ns)
        if best_d is None or d < best_d:
            best_d = d
            best_i = i
    return best_i


def save_jpeg_from_frame(frame_rgb, quality, path):
    # Picamera2 RGB888 在这里按 BGR 排列处理，转成 RGB 再存
    img = Image.fromarray(frame_rgb[:, :, ::-1])
    bio = io.BytesIO()
    img.save(bio, format="JPEG", quality=quality)
    data = bio.getvalue()
    t0 = time.monotonic()
    with open(path, "wb") as f:
        f.write(data)  # 不 fsync
    t1 = time.monotonic()
    return len(data), (t1 - t0) * 1000.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--blocked-level", type=int, choices=[0, 1], default=0)
    ap.add_argument("--debounce-ms", type=float, default=5.0)
    ap.add_argument("--out", default=os.path.expanduser("~/ir_bench_results"))
    ap.add_argument("--shots-per-config", type=int, default=SHOTS_PER_CONFIG)
    args = ap.parse_args()

    blocked = args.blocked_level
    os.makedirs(args.out, exist_ok=True)
    for _, p in STORAGES:
        os.makedirs(p, exist_ok=True)

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    configs = [(res, q, st) for res in RESOLUTIONS for st in STORAGES for q in QUALITIES]
    total_trigger_count = len(configs) * args.shots_per_config
    done = 0

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.out, f"bench_align_{stamp}.csv")
    fields = [
        "config", "res", "quality", "storage", "shot", "variant",
        "trigger_to_frame_ms", "best_frame_age_ms", "sensor_delta_to_trigger_ms",
        "wait_after_trigger_ms", "candidate_count",
        "encode_ms", "write_ms", "trigger_to_file_ms",
        "file_kb", "ring_fps", "ring_size", "cpu_pct", "mem_pct", "temp_c", "throttled", "file"
    ]

    cf = open(csv_path, "w", newline="", encoding="utf-8")
    cw = csv.DictWriter(cf, fieldnames=fields)
    cw.writeheader()

    all_rows = []

    picam2 = None
    ring = None
    cur_res = None

    print(f"共 {len(configs)} 组配置, {total_trigger_count} 次遮挡触发。")
    print(f"每次触发保存 3 张: prev / closest / next")
    print(f"结果 CSV: {csv_path}\n")

    try:
        for ci, ((w, h), q, (sname, sdir)) in enumerate(configs, 1):
            ring_size = ring_size_for_resolution(w, h)
            wait_ms = wait_ms_for_resolution(w, h)

            if cur_res != (w, h):
                if ring:
                    ring.stop()
                if picam2:
                    picam2.stop()
                    picam2.close()

                picam2 = Picamera2()
                cfg = picam2.create_video_configuration(
                    main={"size": (w, h), "format": "RGB888"},
                    buffer_count=max(4, min(8, ring_size // 2))
                )
                picam2.configure(cfg)
                picam2.start()
                time.sleep(1.0)  # AE/AWB 稳定

                ring = FrameRing(picam2, maxlen=ring_size)
                ring.start()
                time.sleep(0.5)
                cur_res = (w, h)

            cname = f"{w}x{h}_q{q}_{sname}"
            print(f"=== 配置 {ci}/{len(configs)}: {cname} | ring={ring_size}, wait={wait_ms:.0f}ms, fps≈{ring.fps:.1f} ===")
            cpu_prev = cpu_times()

            for shot in range(1, args.shots_per_config + 1):
                # 等待恢复到未遮挡，避免连触发
                while GPIO.input(PIN) == blocked:
                    time.sleep(0.003)

                print(f"  [{done+1}/{total_trigger_count}] 请遮挡（本组第 {shot}/{args.shots_per_config} 次）...", flush=True)

                _, cpu_prev = cpu_pct(cpu_prev)

                # 等触发 + 消抖
                while True:
                    while GPIO.input(PIN) != blocked:
                        time.sleep(0.0005)
                    t_det = time.monotonic()
                    t_det_ns = now_ns()

                    ok = True
                    t_end = t_det + args.debounce_ms / 1000.0
                    while time.monotonic() < t_end:
                        if GPIO.input(PIN) != blocked:
                            ok = False
                            break
                        time.sleep(0.0002)
                    if ok:
                        break

                # 关键改动：触发后等待一段时间，再取 ring 快照并做最近匹配
                time.sleep(wait_ms / 1000.0)
                frames = ring.snapshot()

                if not frames:
                    print("    [ERR] ring 为空，跳过")
                    continue

                best_i = pick_closest_index(frames, t_det_ns)
                if best_i is None:
                    print("    [ERR] 未选出最近帧，跳过")
                    continue

                # 选择 prev / closest / next
                indices = [max(0, best_i - 1), best_i, min(len(frames) - 1, best_i + 1)]
                labels = ["prev", "closest", "next"]

                # 先统计最近帧对齐指标
                best_sensor_ts, best_host_ns, _ = frames[best_i]
                best_age_ms = (t_det_ns - best_sensor_ts) / 1e6  # 正值=帧早于触发
                trig_to_frame_ms = (best_host_ns - t_det_ns) / 1e6 + args.debounce_ms

                # 保存3张
                for idx, label in zip(indices, labels):
                    sensor_ts, host_ns, arr = frames[idx]

                    t_enc0 = time.monotonic()
                    fname = f"{cname}_shot{shot}_{label}.jpg"
                    fpath = os.path.join(sdir, fname)
                    size_bytes, write_ms = save_jpeg_from_frame(arr, q, fpath)
                    t_enc1 = time.monotonic()
                    encode_ms = (t_enc1 - t_enc0) * 1000.0 - write_ms

                    cpu, cpu_prev = cpu_pct(cpu_prev)

                    row = {
                        "config": cname,
                        "res": f"{w}x{h}",
                        "quality": q,
                        "storage": sname,
                        "shot": shot,
                        "variant": label,
                        "trigger_to_frame_ms": round(trig_to_frame_ms, 2),
                        "best_frame_age_ms": round(best_age_ms, 2),
                        "sensor_delta_to_trigger_ms": round((sensor_ts - t_det_ns) / 1e6, 2),
                        "wait_after_trigger_ms": round(wait_ms, 1),
                        "candidate_count": len(frames),
                        "encode_ms": round(encode_ms, 2),
                        "write_ms": round(write_ms, 2),
                        "trigger_to_file_ms": round((time.monotonic() - t_det) * 1000.0 + args.debounce_ms, 2),
                        "file_kb": round(size_bytes / 1024.0, 1),
                        "ring_fps": round(ring.fps, 1),
                        "ring_size": ring_size,
                        "cpu_pct": round(cpu, 1),
                        "mem_pct": round(mem_used_pct(), 1),
                        "temp_c": round(temp_c(), 1),
                        "throttled": throttled(),
                        "file": fpath,
                    }
                    cw.writerow(row)
                    cf.flush()
                    all_rows.append(row)

                done += 1
                print(
                    f"      完成: closest帧龄 {best_age_ms:.1f}ms | "
                    f"取帧指标 {trig_to_frame_ms:.1f}ms | 候选{len(frames)}帧 | fps≈{ring.fps:.1f}"
                )

                # 等待遮挡释放
                while GPIO.input(PIN) == blocked:
                    time.sleep(0.003)
                time.sleep(0.25)

    except KeyboardInterrupt:
        print("\n用户中断，输出已完成数据汇总。")
    finally:
        if ring:
            ring.stop()
        if picam2:
            try:
                picam2.stop()
                picam2.close()
            except Exception:
                pass
        GPIO.cleanup()
        cf.close()

    # 汇总（只看 closest）
    closest = [r for r in all_rows if r["variant"] == "closest"]
    if not closest:
        print("没有可汇总数据。")
        return

    groups = {}
    for r in closest:
        groups.setdefault(r["config"], []).append(r)

    summary = []
    for name, rs in groups.items():
        ages = [abs(float(x["best_frame_age_ms"])) for x in rs]
        t2f = [float(x["trigger_to_frame_ms"]) for x in rs]
        t2file = [float(x["trigger_to_file_ms"]) for x in rs]
        fps = [float(x["ring_fps"]) for x in rs]
        summary.append({
            "config": name,
            "age_abs_mean": statistics.mean(ages),
            "age_abs_max": max(ages),
            "t2f_mean": statistics.mean(t2f),
            "t2file_mean": statistics.mean(t2file),
            "fps_mean": statistics.mean(fps),
        })

    print("\n" + "=" * 86)
    print("汇总（仅 closest，按 |帧龄| 均值升序）")
    print("=" * 86)
    print(f"{'配置':<24}{'|帧龄|均值':>10}{'|帧龄|最大':>10}{'取帧均值':>10}{'落盘均值':>10}{'fps':>8}")
    for s in sorted(summary, key=lambda x: x["age_abs_mean"]):
        print(
            f"{s['config']:<24}"
            f"{s['age_abs_mean']:>10.1f}"
            f"{s['age_abs_max']:>10.1f}"
            f"{s['t2f_mean']:>10.1f}"
            f"{s['t2file_mean']:>10.1f}"
            f"{s['fps_mean']:>8.1f}"
        )

    best_align = min(summary, key=lambda x: x["age_abs_mean"])
    best_t2f = min(summary, key=lambda x: x["t2f_mean"])
    print(f"\n最优时间对齐: {best_align['config']} (|帧龄|均值 {best_align['age_abs_mean']:.1f}ms)")
    print(f"最快取帧指标: {best_t2f['config']} (trigger->frame 均值 {best_t2f['t2f_mean']:.1f}ms)")
    print(f"\n详细 CSV: {csv_path}")
    print("图片目录: SD=~/ir_bench, tmpfs=/dev/shm/ir_bench (tmpfs 重启即丢)")

if __name__ == "__main__":
    main()
