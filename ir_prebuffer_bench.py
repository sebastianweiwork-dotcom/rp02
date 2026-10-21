#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
预缓冲极限性能测试 (Pi Zero 2 W + OV5647 + 红外对射 GPIO17)

矩阵: 分辨率(640x480, 1296x972, 1920x1080) x JPEG质量(70, 90) x 输出介质(SD卡, tmpfs)
      = 12 组, 每组手动遮挡 3 次 = 36 次遮挡。fsync 关闭。

原理: 相机常驻, 后台线程持续把最新几帧放进内存环形缓冲。
      触发时直接取缓冲里最新一帧(不等新帧), 再编码、写盘。

指标:
  trigger_to_frame_ms : 触发检测 -> 拿到帧(含消抖)
  frame_age_ms        : 该帧曝光起点距触发的时间(正数=帧比触发早, 负数=帧晚于触发)
  encode_ms / write_ms
  trigger_to_file_ms  : 触发检测 -> 文件写完
运行: sudo python3 ir_prebuffer_bench.py [--blocked-level 0]
"""
import os, io, csv, time, shutil, argparse, threading, subprocess, statistics
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
RING = 4


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


def mem_used_pct():
    d = {}
    with open("/proc/meminfo") as f:
        for l in f:
            k, v = l.split(":")
            d[k] = int(v.split()[0])
    return 100.0 * (d["MemTotal"] - d["MemAvailable"]) / d["MemTotal"]


def temp():
    try:
        return int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000
    except Exception:
        return float("nan")


def throttled():
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                             text=True, timeout=2).stdout.strip()
        return out.split("=")[-1]
    except Exception:
        return "n/a"


class Ring:
    """相机常驻 + 后台线程持续写入环形缓冲"""
    def __init__(self, picam2):
        self.cam = picam2
        self.buf = deque(maxlen=RING)
        self.lock = threading.Lock()
        self.run = False
        self.th = None
        self.fps = 0.0

    def start(self):
        self.run = True
        self.th = threading.Thread(target=self._loop, daemon=True)
        self.th.start()

    def stop(self):
        self.run = False
        if self.th:
            self.th.join(timeout=3)
        self.buf.clear()

    def _loop(self):
        last = time.monotonic()
        while self.run:
            try:
                req = self.cam.capture_request()
            except Exception:
                break
            try:
                arr = req.make_array("main")
                ts = req.get_metadata().get("SensorTimestamp", now_ns())
            finally:
                req.release()
            with self.lock:
                self.buf.append((ts, arr))
            t = time.monotonic()
            self.fps = 0.9 * self.fps + 0.1 / max(t - last, 1e-6)
            last = t

    def latest(self):
        with self.lock:
            return self.buf[-1] if self.buf else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--blocked-level", type=int, choices=[0, 1], default=0)
    ap.add_argument("--debounce-ms", type=float, default=5.0)
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
    total = len(configs) * SHOTS_PER_CONFIG
    done = 0
    rows = []
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.out, f"bench_{stamp}.csv")
    fields = ["config", "res", "quality", "storage", "shot", "trigger_to_frame_ms",
              "frame_age_ms", "encode_ms", "write_ms", "trigger_to_file_ms",
              "file_kb", "ring_fps", "cpu_pct", "mem_pct", "temp_c", "throttled", "file"]
    cf = open(csv_path, "w", newline="", encoding="utf-8")
    cw = csv.DictWriter(cf, fieldnames=fields)
    cw.writeheader()

    print(f"共 {len(configs)} 组配置, {total} 次遮挡。结果: {csv_path}\n")
    picam2 = None
    cur_res = None
    ring = None
    try:
        for ci, ((w, h), q, (sname, sdir)) in enumerate(configs, 1):
            if cur_res != (w, h):
                if ring:
                    ring.stop()
                if picam2:
                    picam2.stop()
                    picam2.close()
                picam2 = Picamera2()
                cfg = picam2.create_video_configuration(
                    main={"size": (w, h), "format": "RGB888"}, buffer_count=4)
                picam2.configure(cfg)
                picam2.start()
                time.sleep(1.0)  # AE/AWB 稳定
                ring = Ring(picam2)
                ring.start()
                cur_res = (w, h)
                time.sleep(0.5)
            name = f"{w}x{h}_q{q}_{sname}"
            print(f"\n=== 配置 {ci}/{len(configs)}: {name} (环缓冲帧率约 {ring.fps:.1f}fps) ===")
            cpu_prev = cpu_times()

            for k in range(1, SHOTS_PER_CONFIG + 1):
                # 等未遮挡
                while GPIO.input(PIN) == blocked:
                    time.sleep(0.005)
                print(f"  [{done + 1}/{total}] 请遮挡对射 (本组第 {k}/{SHOTS_PER_CONFIG} 次) ...", flush=True)
                _, cpu_prev = cpu_pct(cpu_prev)
                while True:
                    while GPIO.input(PIN) != blocked:
                        time.sleep(0.0005)
                    t_det = time.monotonic()
                    t_det_ns = now_ns()
                    ok = True
                    while time.monotonic() - t_det < args.debounce_ms / 1000:
                        if GPIO.input(PIN) != blocked:
                            ok = False
                            break
                        time.sleep(0.0002)
                    if ok:
                        break

                item = ring.latest()
                t_frame = time.monotonic()
                if item is None:
                    print("  [ERR] 缓冲为空, 跳过")
                    continue
                ts_ns, arr = item

                t2 = time.monotonic()
                img = Image.fromarray(arr[:, :, ::-1])  # picamera2 的 RGB888 实为 BGR 排列
                bio = io.BytesIO()
                img.save(bio, format="JPEG", quality=q)
                data = bio.getvalue()
                t3 = time.monotonic()

                fpath = os.path.join(sdir, f"{name}_{k}.jpg")
                with open(fpath, "wb") as f:
                    f.write(data)  # 不 fsync
                t4 = time.monotonic()

                c, cpu_prev = cpu_pct(cpu_prev)
                row = {
                    "config": name, "res": f"{w}x{h}", "quality": q, "storage": sname, "shot": k,
                    "trigger_to_frame_ms": round((t_frame - t_det) * 1000 + args.debounce_ms, 2),
                    "frame_age_ms": round((t_det_ns - ts_ns) / 1e6, 1),
                    "encode_ms": round((t3 - t2) * 1000, 1),
                    "write_ms": round((t4 - t3) * 1000, 1),
                    "trigger_to_file_ms": round((t4 - t_det) * 1000 + args.debounce_ms, 1),
                    "file_kb": round(len(data) / 1024, 1),
                    "ring_fps": round(ring.fps, 1),
                    "cpu_pct": round(c, 1), "mem_pct": round(mem_used_pct(), 1),
                    "temp_c": round(temp(), 1), "throttled": throttled(), "file": fpath,
                }
                rows.append(row)
                cw.writerow(row)
                cf.flush()
                done += 1
                print(f"      完成: 取帧 {row['trigger_to_frame_ms']}ms | 帧龄 {row['frame_age_ms']}ms | "
                      f"编码 {row['encode_ms']}ms | 写盘 {row['write_ms']}ms | "
                      f"总 {row['trigger_to_file_ms']}ms | {row['file_kb']}KB | "
                      f"CPU {row['cpu_pct']}% | {row['temp_c']}°C | {row['throttled']}")
                while GPIO.input(PIN) == blocked:
                    time.sleep(0.005)
                time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n用户中断, 输出已完成部分的报告")
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

    # ---------- 汇总报告 ----------
    if not rows:
        return
    groups = {}
    for r in rows:
        groups.setdefault(r["config"], []).append(r)
    summ = []
    for name, rs in groups.items():
        f = lambda key: [x[key] for x in rs]
        summ.append({
            "config": name,
            "frame_mean": statistics.mean(f("trigger_to_frame_ms")),
            "age_mean": statistics.mean(f("frame_age_ms")),
            "file_mean": statistics.mean(f("trigger_to_file_ms")),
            "file_max": max(f("trigger_to_file_ms")),
            "fps": statistics.mean(f("ring_fps")),
            "kb": statistics.mean(f("file_kb")),
        })
    print("\n" + "=" * 78)
    print("汇总 (每组 3 张的平均值)")
    print("=" * 78)
    print(f"{'配置':<22}{'取帧ms':>8}{'帧龄ms':>8}{'落盘均值':>9}{'落盘最大':>9}{'fps':>6}{'KB':>7}")
    for s in sorted(summ, key=lambda x: x["file_mean"]):
        print(f"{s['config']:<22}{s['frame_mean']:>8.1f}{s['age_mean']:>8.1f}"
              f"{s['file_mean']:>9.1f}{s['file_max']:>9.1f}{s['fps']:>6.1f}{s['kb']:>7.0f}")
    b1 = min(summ, key=lambda x: x["frame_mean"])
    b2 = min(summ, key=lambda x: x["file_mean"])
    b3 = min(summ, key=lambda x: abs(x["age_mean"]))
    print(f"\n最快 触发->拿到帧 : {b1['config']} ({b1['frame_mean']:.1f}ms)")
    print(f"最快 触发->文件   : {b2['config']} ({b2['file_mean']:.1f}ms)")
    print(f"画面最贴近触发瞬间: {b3['config']} (帧龄 {b3['age_mean']:.1f}ms)")
    print(f"\n详细数据: {csv_path}")
    print("照片: SD卡 ~/ir_bench, tmpfs /dev/shm/ir_bench (tmpfs 重启即丢, 请及时拷走)")


if __name__ == "__main__":
    main()
