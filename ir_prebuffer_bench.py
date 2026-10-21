#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
预缓冲时间对齐基准 (Pi Zero 2 W + OV5647 + 红外对射 GPIO17) - 稳健版(修复YUV stride)

目标：
- 优先优化“画面时间尽可能接近触发瞬间”
- 触发后等待少量时间，从环形缓冲按 SensorTimestamp 选最接近触发时刻的帧
- 采帧与JPEG编码分离：触发阶段只取帧和记时间，组末再离线编码保存，避免干扰时间对齐

矩阵：
  分辨率: 640x480, 1296x972, 1920x1080
  JPEG质量: 70, 90
  存储: SD卡(~/ir_bench), tmpfs(/dev/shm/ir_bench)
  每组触发: 3 次
总触发: 12 组 * 3 = 36 次

运行：
  sudo python3 ir_prebuffer_bench.py --blocked-level 0
"""

import os
import io
import csv
import gc
import time
import argparse
import threading
import subprocess
import statistics
from collections import deque
from datetime import datetime

import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from PIL import Image

PIN = 17
RESOLUTIONS = [(640, 480), (1296, 972), (1920, 1080)]
QUALITIES = [70, 90]
STORAGES = [("sd", os.path.expanduser("~/ir_bench")), ("tmpfs", "/dev/shm/ir_bench")]
SHOTS = 3

# 按Zero2W内存/负载设定
RING = {(640, 480): 24, (1296, 972): 8, (1920, 1080): 6}
WAIT_MS = {(640, 480): 35.0, (1296, 972): 45.0, (1920, 1080): 55.0}
STALE_MS = 100.0  # 触发时若最后一帧距离当前超过该值, 视为采帧停滞


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


def mem_pct():
    d = {}
    with open("/proc/meminfo", "r", encoding="utf-8") as f:
        for line in f:
            k, v = line.split(":")
            d[k] = int(v.split()[0])
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


class Ring:
    """
    采帧线程：只存引用和时间戳，不做编码
    缓冲项: (sensor_ts_ns, host_ts_ns, ndarray_yuv420)
    """
    def __init__(self, cam, n):
        self.cam = cam
        self.buf = deque(maxlen=n)
        self.lock = threading.Lock()
        self.running = False
        self.th = None
        self.fps = 0.0
        self.last_host_ns = 0

    def start(self):
        self.running = True
        self.th = threading.Thread(target=self._loop, daemon=True)
        self.th.start()

    def stop(self):
        self.running = False
        if self.th:
            self.th.join(timeout=2)
        with self.lock:
            self.buf.clear()

    def _loop(self):
        last = time.monotonic()
        while self.running:
            try:
                req = self.cam.capture_request(wait=True)
            except Exception:
                break

            host_ns = now_ns()
            try:
                md = req.get_metadata() or {}
                sensor_ts = int(md.get("SensorTimestamp", host_ns))
                # 关键：release前copy，否则底层buffer会被复用
                arr = req.make_array("main").copy()
            finally:
                req.release()

            with self.lock:
                self.buf.append((sensor_ts, host_ns, arr))
                self.last_host_ns = host_ns

            cur = time.monotonic()
            inst = 1.0 / max(cur - last, 1e-6)
            self.fps = inst if self.fps == 0 else 0.9 * self.fps + 0.1 * inst
            last = cur

    def snapshot(self):
        with self.lock:
            return list(self.buf)


def yuv420_to_jpeg(arr, w, h, q):
    """
    YUV420(I420) -> JPEG
    修复点：按stride解析，不假设stride==width

    arr.shape = (h*3/2, stride)
      - Y: h 行, 每行 stride 字节，取前 w
      - UV 区域: 后 h/2 行，按平面展开
    """
    stride = arr.shape[1]

    # Y plane
    y = np.ascontiguousarray(arr[:h, :w])

    # UV planes as flat bytes
    uv_flat = arr[h:, :].reshape(-1)

    half_stride = stride // 2
    plane_len = (h // 2) * half_stride  # U 或 V 平面的字节数（带stride）

    if uv_flat.size < plane_len * 2:
        raise ValueError(
            f"UV buffer too small: size={uv_flat.size}, need={plane_len*2}, "
            f"shape={arr.shape}, w={w}, h={h}, stride={stride}"
        )

    u = uv_flat[:plane_len].reshape(h // 2, half_stride)[:, :w // 2]
    v = uv_flat[plane_len:plane_len * 2].reshape(h // 2, half_stride)[:, :w // 2]

    Y = Image.fromarray(y, "L")
    U = Image.fromarray(np.ascontiguousarray(u), "L").resize((w, h), Image.BILINEAR)
    V = Image.fromarray(np.ascontiguousarray(v), "L").resize((w, h), Image.BILINEAR)

    img = Image.merge("YCbCr", (Y, U, V)).convert("RGB")
    bio = io.BytesIO()
    img.save(bio, format="JPEG", quality=q)
    return bio.getvalue()


def choose_closest(frames, trigger_ns):
    return min(range(len(frames)), key=lambda i: abs(frames[i][0] - trigger_ns))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--blocked-level", type=int, choices=[0, 1], default=0,
                    help="遮挡时GPIO电平，默认0")
    ap.add_argument("--debounce-ms", type=float, default=3.0,
                    help="消抖时长，默认3ms")
    ap.add_argument("--out", default=os.path.expanduser("~/ir_bench_results"),
                    help="CSV输出目录")
    ap.add_argument("--force-fps", type=float, default=0.0,
                    help="可选：强制帧率，如60。0表示不强制")
    args = ap.parse_args()

    blocked = args.blocked_level

    os.makedirs(args.out, exist_ok=True)
    for _, p in STORAGES:
        os.makedirs(p, exist_ok=True)

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    configs = [(r, q, s) for r in RESOLUTIONS for s in STORAGES for q in QUALITIES]
    total = len(configs) * SHOTS
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.out, f"bench_align_{stamp}.csv")

    fields = [
        "config", "res", "quality", "storage", "shot",
        "frame_age_ms", "trigger_to_frame_ms",
        "wait_ms", "ring_n", "ring_fps",
        "cpu_pct", "mem_pct", "temp_c", "throttled",
        "encode_ms", "write_ms", "file_kb", "file",
        "valid"
    ]

    cf = open(csv_path, "w", newline="", encoding="utf-8")
    cw = csv.DictWriter(cf, fieldnames=fields)
    cw.writeheader()

    print(f"共 {len(configs)} 组, {total} 次遮挡。CSV: {csv_path}\n")

    rows = []
    cam = None
    ring = None
    cur_res = None
    done = 0

    try:
        for ci, ((w, h), q, (sname, sdir)) in enumerate(configs, 1):
            if cur_res != (w, h):
                if ring:
                    ring.stop()
                if cam:
                    try:
                        cam.stop()
                        cam.close()
                    except Exception:
                        pass
                    cam = None
                gc.collect()

                print("  (切换分辨率, 冷却 3 秒...)")
                time.sleep(3)

                cam = Picamera2()
                controls = {}
                if args.force_fps > 0:
                    controls["FrameRate"] = args.force_fps

                cfg = cam.create_video_configuration(
                    main={"size": (w, h), "format": "YUV420"},
                    buffer_count=4,
                    controls=controls if controls else None
                )
                cam.configure(cfg)
                cam.start()
                time.sleep(1.0)

                ring = Ring(cam, RING[(w, h)])
                ring.start()
                time.sleep(0.5)
                cur_res = (w, h)

            name = f"{w}x{h}_q{q}_{sname}"
            wait_ms = WAIT_MS[(w, h)]
            print(f"=== 配置 {ci}/{len(configs)}: {name} | ring={RING[(w,h)]} wait={wait_ms:.0f}ms fps≈{ring.fps:.1f} ===")

            pending = []  # 本组先收集触发结果，后续统一离线编码
            cpu_prev = cpu_times()
            shot = 0

            while shot < SHOTS:
                while GPIO.input(PIN) == blocked:
                    time.sleep(0.003)

                print(f"  [{done+1}/{total}] 请遮挡 (本组第 {shot+1}/{SHOTS} 次) ...", flush=True)
                _, cpu_prev = cpu_pct(cpu_prev)

                # 等待稳定遮挡
                while True:
                    while GPIO.input(PIN) != blocked:
                        time.sleep(0.0005)
                    t_det = time.monotonic()
                    t_det_ns = now_ns()

                    stable = True
                    t_end = t_det + args.debounce_ms / 1000.0
                    while time.monotonic() < t_end:
                        if GPIO.input(PIN) != blocked:
                            stable = False
                            break
                        time.sleep(0.0002)
                    if stable:
                        break

                stale_before = (t_det_ns - ring.last_host_ns) / 1e6 if ring.last_host_ns else 1e9

                time.sleep(wait_ms / 1000.0)
                frames = ring.snapshot()
                now_pick_ns = now_ns()
                stale_after = (now_pick_ns - ring.last_host_ns) / 1e6 if ring.last_host_ns else 1e9

                if (not frames) or stale_before > STALE_MS or stale_after > STALE_MS:
                    print(f"      [无效] 采帧停滞 (before={stale_before:.1f}ms, after={stale_after:.1f}ms), 请重试")
                    while GPIO.input(PIN) == blocked:
                        time.sleep(0.003)
                    time.sleep(0.3)
                    continue

                bi = choose_closest(frames, t_det_ns)
                sensor_ts, host_ts, arr = frames[bi]

                age_ms = (t_det_ns - sensor_ts) / 1e6   # 正=帧早于触发
                t2f_ms = (host_ts - t_det_ns) / 1e6     # 触发->该帧到达主机（可为负/正）

                c, cpu_prev = cpu_pct(cpu_prev)
                m = mem_pct()
                t = temp_c()
                th = throttled()

                pending.append({
                    "shot": shot + 1,
                    "arr": arr,
                    "age_ms": age_ms,
                    "t2f_ms": t2f_ms,
                    "ring_fps": ring.fps,
                    "cpu": c,
                    "mem": m,
                    "temp": t,
                    "thr": th,
                    "valid": 1
                })

                shot += 1
                done += 1
                print(
                    f"      完成: 帧龄 {age_ms:+.1f}ms | 取帧 {t2f_ms:.1f}ms | 候选 {len(frames)} 帧 | "
                    f"CPU {c:.0f}% | {t:.1f}°C"
                )

                while GPIO.input(PIN) == blocked:
                    time.sleep(0.003)
                time.sleep(0.2)

            # 离线编码保存（不影响触发路径）
            print("  (离线编码保存中...)")
            for p in pending:
                fpath = os.path.join(sdir, f"{name}_{p['shot']}.jpg")
                enc_ms = 0.0
                wr_ms = 0.0
                kb = 0.0
                ok = 1

                try:
                    t0 = time.monotonic()
                    data = yuv420_to_jpeg(p["arr"], w, h, q)
                    t1 = time.monotonic()
                    with open(fpath, "wb") as f:
                        f.write(data)  # 不 fsync
                    t2 = time.monotonic()

                    enc_ms = (t1 - t0) * 1000.0
                    wr_ms = (t2 - t1) * 1000.0
                    kb = len(data) / 1024.0
                except Exception as e:
                    ok = 0
                    print(f"    [编码失败] {name} shot{p['shot']}: {e}")
                    fpath = ""

                row = {
                    "config": name,
                    "res": f"{w}x{h}",
                    "quality": q,
                    "storage": sname,
                    "shot": p["shot"],
                    "frame_age_ms": round(p["age_ms"], 2),
                    "trigger_to_frame_ms": round(p["t2f_ms"], 2),
                    "wait_ms": wait_ms,
                    "ring_n": RING[(w, h)],
                    "ring_fps": round(p["ring_fps"], 1),
                    "cpu_pct": round(p["cpu"], 1),
                    "mem_pct": round(p["mem"], 1),
                    "temp_c": round(p["temp"], 1),
                    "throttled": p["thr"],
                    "encode_ms": round(enc_ms, 1),
                    "write_ms": round(wr_ms, 1),
                    "file_kb": round(kb, 1),
                    "file": fpath,
                    "valid": ok
                }
                cw.writerow(row)
                cf.flush()
                rows.append(row)

            pending.clear()
            gc.collect()

    except KeyboardInterrupt:
        print("\n用户中断，输出已完成部分汇总。")
    finally:
        if ring:
            ring.stop()
        if cam:
            try:
                cam.stop()
                cam.close()
            except Exception:
                pass
        GPIO.cleanup()
        cf.close()

    valid_rows = [r for r in rows if r["valid"] == 1]
    if not valid_rows:
        print("无有效数据。")
        return

    # 按配置汇总
    groups = {}
    for r in valid_rows:
        groups.setdefault(r["config"], []).append(r)

    print("\n" + "=" * 90)
    print("汇总（目标：|帧龄|越小越好）")
    print("=" * 90)
    print(f"{'配置':<24}{'|帧龄|均值':>11}{'|帧龄|最大':>11}{'帧龄均值(带符号)':>16}{'fps':>8}{'CPU%':>8}")

    summary = []
    for name, rs in groups.items():
        ages = [float(x["frame_age_ms"]) for x in rs]
        abs_mean = statistics.mean(abs(a) for a in ages)
        abs_max = max(abs(a) for a in ages)
        signed_mean = statistics.mean(ages)
        fps_mean = statistics.mean(float(x["ring_fps"]) for x in rs)
        cpu_mean = statistics.mean(float(x["cpu_pct"]) for x in rs)
        summary.append((name, abs_mean, abs_max, signed_mean, fps_mean, cpu_mean))

    for s in sorted(summary, key=lambda x: x[1]):
        print(f"{s[0]:<24}{s[1]:>11.1f}{s[2]:>11.1f}{s[3]:>16.1f}{s[4]:>8.1f}{s[5]:>8.1f}")

    best = min(summary, key=lambda x: x[1])
    print(f"\n最佳时间对齐: {best[0]}  (|帧龄|均值 {best[1]:.1f} ms)")
    print(f"CSV: {csv_path}")
    print("照片目录: SD=~/ir_bench, tmpfs=/dev/shm/ir_bench (tmpfs 重启会丢失)")


if __name__ == "__main__":
    main()
