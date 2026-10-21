#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
预缓冲时间对齐基准 (Pi Zero 2 W + OV5647 + 红外对射 GPIO17) - 稳健版

实时路径: 检测触发 -> 等待 N ms -> 在缓冲里按 SensorTimestamp 选最近帧(只复制引用)
离线路径: 触发结束后再把选中帧编码为 JPEG 并写盘(不干扰采帧)

矩阵: 分辨率(640x480, 1296x972, 1920x1080) x 质量(70,90) x 存储(sd,tmpfs) x 3 次遮挡
运行: sudo python3 ir_prebuffer_bench.py --blocked-level 0
"""
import os, io, csv, time, gc, argparse, threading, subprocess, statistics
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
RING = {(640, 480): 24, (1296, 972): 8, (1920, 1080): 6}
WAIT_MS = {(640, 480): 35.0, (1296, 972): 45.0, (1920, 1080): 55.0}
STALE_MS = 100.0  # 触发时最新帧比触发早超过该值 -> 无效


def now_ns():
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)


def cpu_times():
    with open("/proc/stat") as f:
        v = list(map(int, f.readline().split()[1:]))
    return v[3] + (v[4] if len(v) > 4 else 0), sum(v)


def cpu_pct(prev):
    i, t = cpu_times()
    dt = t - prev[1]
    return (100.0 * (1 - (i - prev[0]) / dt) if dt > 0 else 0.0), (i, t)


def mem_pct():
    d = {}
    with open("/proc/meminfo") as f:
        for l in f:
            k, v = l.split(":")
            d[k] = int(v.split()[0])
    return 100.0 * (d["MemTotal"] - d["MemAvailable"]) / d["MemTotal"]


def temp_c():
    try:
        return int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000
    except Exception:
        return float("nan")


def throttled():
    try:
        o = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                           text=True, timeout=2).stdout.strip()
        return o.split("=")[-1]
    except Exception:
        return "n/a"


class Ring:
    """只存引用: (sensor_ts, host_ns, ndarray)。不转色,不编码。"""
    def __init__(self, cam, n):
        self.cam, self.buf = cam, deque(maxlen=n)
        self.lock = threading.Lock()
        self.run = False
        self.fps = 0.0
        self.last_host = 0
        self.th = None

    def start(self):
        self.run = True
        self.th = threading.Thread(target=self._loop, daemon=True)
        self.th.start()

    def stop(self):
        self.run = False
        if self.th:
            self.th.join(timeout=2)
        with self.lock:
            self.buf.clear()

    def _loop(self):
        last = time.monotonic()
        while self.run:
            try:
                req = self.cam.capture_request(wait=True)
            except Exception:
                break
            host = now_ns()
            try:
                md = req.get_metadata() or {}
                ts = int(md.get("SensorTimestamp", host))
                arr = req.make_array("main").copy()  # 必须拷贝: release 后缓冲会被复用
            finally:
                req.release()
            with self.lock:
                self.buf.append((ts, host, arr))
                self.last_host = host
            t = time.monotonic()
            inst = 1.0 / max(t - last, 1e-6)
            self.fps = inst if self.fps == 0 else 0.9 * self.fps + 0.1 * inst
            last = t

    def snap(self):
        with self.lock:
            return list(self.buf)


def yuv420_to_jpeg(arr, w, h, q):
    """YUV420(I420, 形状 (h*3/2, w)) -> 仅取亮度+粗略转换; 用 Pillow 的 YCbCr 合成"""
    y = arr[:h, :w]
    u = arr[h:h + h // 4, :].reshape(h // 2, w // 2)
    v = arr[h + h // 4:, :].reshape(h // 2, w // 2)
    Y = Image.fromarray(y, "L")
    U = Image.fromarray(np.ascontiguousarray(u), "L").resize((w, h), Image.BILINEAR)
    V = Image.fromarray(np.ascontiguousarray(v), "L").resize((w, h), Image.BILINEAR)
    img = Image.merge("YCbCr", (Y, U, V)).convert("RGB")
    bio = io.BytesIO()
    img.save(bio, "JPEG", quality=q)
    return bio.getvalue()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--blocked-level", type=int, choices=[0, 1], default=0)
    ap.add_argument("--debounce-ms", type=float, default=3.0)
    ap.add_argument("--out", default=os.path.expanduser("~/ir_bench_results"))
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
    fields = ["config", "res", "quality", "storage", "shot",
              "frame_age_ms", "trigger_to_frame_ms", "wait_ms", "ring_n", "ring_fps",
              "cpu_pct", "mem_pct", "temp_c", "throttled",
              "encode_ms", "write_ms", "file_kb", "file"]
    cf = open(csv_path, "w", newline="", encoding="utf-8")
    cw = csv.DictWriter(cf, fieldnames=fields)
    cw.writeheader()
    print(f"共 {len(configs)} 组, {total} 次遮挡。CSV: {csv_path}\n")

    rows = []
    cam = None
    ring = None
    cur = None
    done = 0
    try:
        for ci, ((w, h), q, (sname, sdir)) in enumerate(configs, 1):
            if cur != (w, h):
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
                cfg = cam.create_video_configuration(
                    main={"size": (w, h), "format": "YUV420"}, buffer_count=4)
                cam.configure(cfg)
                cam.start()
                time.sleep(1.0)
                ring = Ring(cam, RING[(w, h)])
                ring.start()
                time.sleep(0.5)
                cur = (w, h)

            name = f"{w}x{h}_q{q}_{sname}"
            wait_ms = WAIT_MS[(w, h)]
            print(f"=== 配置 {ci}/{len(configs)}: {name} | ring={RING[(w,h)]} wait={wait_ms:.0f}ms fps≈{ring.fps:.1f} ===")
            pending = []  # 本组选中的帧, 稍后离线编码
            cpu_prev = cpu_times()

            shot = 0
            while shot < SHOTS:
                while GPIO.input(PIN) == blocked:
                    time.sleep(0.003)
                print(f"  [{done+1}/{total}] 请遮挡 (本组第 {shot+1}/{SHOTS} 次) ...", flush=True)
                _, cpu_prev = cpu_pct(cpu_prev)

                while True:
                    while GPIO.input(PIN) != blocked:
                        time.sleep(0.0005)
                    t_det = time.monotonic()
                    t_ns = now_ns()
                    ok = True
                    while time.monotonic() - t_det < args.debounce_ms / 1000:
                        if GPIO.input(PIN) != blocked:
                            ok = False
                            break
                        time.sleep(0.0002)
                    if ok:
                        break

                # 看门狗: 触发瞬间缓冲是否还在更新
                stale = (t_ns - ring.last_host) / 1e6 if ring.last_host else 1e9
                time.sleep(wait_ms / 1000)
                frames = ring.snap()
                t_pick_ns = now_ns()
                fresh = (t_pick_ns - ring.last_host) / 1e6

                if not frames or fresh > STALE_MS or stale > STALE_MS:
                    print(f"      [无效] 采帧停滞 (距上帧 {fresh:.0f}ms), 请移开后重试")
                    while GPIO.input(PIN) == blocked:
                        time.sleep(0.003)
                    time.sleep(0.5)
                    continue

                bi = min(range(len(frames)), key=lambda i: abs(frames[i][0] - t_ns))
                ts, host, arr = frames[bi]
                age = (t_ns - ts) / 1e6  # 正=帧早于触发, 负=帧晚于触发
                t2f = (host - t_ns) / 1e6
                c, cpu_prev = cpu_pct(cpu_prev)

                pending.append({
                    "arr": arr, "shot": shot + 1, "age": age, "t2f": t2f,
                    "cpu": c, "mem": mem_pct(), "temp": temp_c(), "thr": throttled(),
                    "fps": ring.fps,
                })
                shot += 1
                done += 1
                print(f"      完成: 帧龄 {age:+.1f}ms | 取帧 {t2f:.1f}ms | 候选 {len(frames)} 帧 | "
                      f"CPU {c:.0f}% | {temp_c():.1f}°C")
                while GPIO.input(PIN) == blocked:
                    time.sleep(0.003)
                time.sleep(0.2)

            # ---- 离线编码(不影响测量) ----
            print("  (离线编码保存中...)")
            for p in pending:
                t0 = time.monotonic()
                data = yuv420_to_jpeg(p["arr"], w, h, q)
                t1 = time.monotonic()
                fpath = os.path.join(sdir, f"{name}_{p['shot']}.jpg")
                with open(fpath, "wb") as f:
                    f.write(data)
                t2 = time.monotonic()
                row = {
                    "config": name, "res": f"{w}x{h}", "quality": q, "storage": sname,
                    "shot": p["shot"], "frame_age_ms": round(p["age"], 2),
                    "trigger_to_frame_ms": round(p["t2f"], 2), "wait_ms": wait_ms,
                    "ring_n": RING[(w, h)], "ring_fps": round(p["fps"], 1),
                    "cpu_pct": round(p["cpu"], 1), "mem_pct": round(p["mem"], 1),
                    "temp_c": round(p["temp"], 1), "throttled": p["thr"],
                    "encode_ms": round((t1 - t0) * 1000, 1),
                    "write_ms": round((t2 - t1) * 1000, 1),
                    "file_kb": round(len(data) / 1024, 1), "file": fpath,
                }
                cw.writerow(row)
                cf.flush()
                rows.append(row)
            pending.clear()
            gc.collect()

    except KeyboardInterrupt:
        print("\n用户中断, 输出已完成部分汇总")
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

    if not rows:
        print("无数据")
        return
    g = {}
    for r in rows:
        g.setdefault(r["config"], []).append(r)
    print("\n" + "=" * 84)
    print("汇总 (|帧龄| 越小, 画面越贴近触发瞬间)")
    print("=" * 84)
    print(f"{'配置':<22}{'|帧龄|均值':>10}{'|帧龄|最大':>10}{'带符号均值':>11}{'fps':>7}{'CPU%':>7}{'编码ms':>9}")
    summ = []
    for n, rs in g.items():
        ages = [r["frame_age_ms"] for r in rs]
        summ.append((n, statistics.mean(abs(a) for a in ages), max(abs(a) for a in ages),
                     statistics.mean(ages), statistics.mean(r["ring_fps"] for r in rs),
                     statistics.mean(r["cpu_pct"] for r in rs),
                     statistics.mean(r["encode_ms"] for r in rs)))
    for s in sorted(summ, key=lambda x: x[1]):
        print(f"{s[0]:<22}{s[1]:>10.1f}{s[2]:>10.1f}{s[3]:>11.1f}{s[4]:>7.1f}{s[5]:>7.0f}{s[6]:>9.0f}")
    best = min(summ, key=lambda x: x[1])
    print(f"\n最佳时间对齐: {best[0]} (|帧龄|均值 {best[1]:.1f}ms)")
    print(f"CSV: {csv_path}")
    print("照片: SD=~/ir_bench, tmpfs=/dev/shm/ir_bench (tmpfs 重启即丢)")


if __name__ == "__main__":
    main()
