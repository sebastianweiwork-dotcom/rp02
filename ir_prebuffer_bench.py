#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
拍照环节基准 (Pi Zero 2 W + OV5647, 红外对射/光耦触发 GPIO17), 1080p

链路: 相机常开 -> 锁定ISP -> YUV420 Ring -> GPIO触发
      -> 自适应等待(出现触发后的第一帧, 有上限) -> 在前后帧里选最近(含偏移补偿)
      -> 保存 YUV420(.npy) + JSON元数据。 JPEG/矫正/上传均为后处理, 不在此脚本内。

运行:
  sudo python3 ir_prebuffer_bench.py --blocked-level 0 --shots 20
  # 先 offset=0 跑一轮, 看汇总的"带符号均值", 再 --offset-ms <该值> 复测
"""
import os, gc, json, time, argparse, threading, statistics, subprocess, csv
from collections import deque
from datetime import datetime

import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2

PIN = 17
SIZE = (1920, 1080)
RING_N = 6
STALE_MS = 100.0


def now_ns():
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)


def temp_c():
    try:
        return int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000.0
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
    """采帧线程: 只存 (sensor_ts, host_ns, ndarray)。不转色, 不编码。"""
    def __init__(self, cam, n):
        self.cam = cam
        self.buf = deque(maxlen=n)
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.running = False
        self.th = None
        self.last_host_ns = 0
        self.ts_hist = deque(maxlen=60)  # 用于计算传感器真实帧率

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
        while self.running:
            try:
                req = self.cam.capture_request(wait=True)
            except Exception:
                break
            host = now_ns()
            try:
                md = req.get_metadata() or {}
                ts = int(md.get("SensorTimestamp", host))
                arr = req.make_array("main").copy()  # release 前必须拷贝
            finally:
                req.release()
            with self.cond:
                self.buf.append((ts, host, arr))
                self.last_host_ns = host
                self.ts_hist.append(ts)
                self.cond.notify_all()

    def snapshot(self):
        with self.lock:
            return list(self.buf)

    def sensor_fps(self):
        with self.lock:
            t = list(self.ts_hist)
        if len(t) < 3:
            return 0.0
        d = np.diff(t)
        return 1e9 / float(np.mean(d))

    def wait_first_after(self, trigger_ns, offset_ns, timeout_s):
        """等待出现 (sensor_ts + offset) > trigger 的帧; 超时返回 False"""
        end = time.monotonic() + timeout_s
        with self.cond:
            while True:
                if self.buf and (self.buf[-1][0] + offset_ns) > trigger_ns:
                    return True
                rem = end - time.monotonic()
                if rem <= 0:
                    return False
                self.cond.wait(rem)


def lock_isp(cam, settle_s=2.0):
    """自动曝光预热后, 读出参数并锁死 AE/AWB"""
    time.sleep(settle_s)
    md = cam.capture_metadata()
    exp = int(md["ExposureTime"])
    gain = float(md["AnalogueGain"])
    cg = md.get("ColourGains", (1.0, 1.0))
    cam.set_controls({
        "AeEnable": False, "AwbEnable": False,
        "ExposureTime": exp, "AnalogueGain": gain,
        "ColourGains": (float(cg[0]), float(cg[1])),
    })
    time.sleep(0.5)
    return {"ExposureTime": exp, "AnalogueGain": gain,
            "ColourGains": [float(cg[0]), float(cg[1])]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--blocked-level", type=int, choices=[0, 1], default=0)
    ap.add_argument("--debounce-ms", type=float, default=3.0)
    ap.add_argument("--shots", type=int, default=20)
    ap.add_argument("--fps", type=float, default=0.0, help="0=默认30fps")
    ap.add_argument("--offset-ms", type=float, default=0.0,
                    help="选帧补偿: 用 ts+offset 与触发比较")
    ap.add_argument("--max-wait-ms", type=float, default=120.0)
    ap.add_argument("--no-lock", action="store_true", help="不锁定ISP")
    ap.add_argument("--save-frames", action="store_true", help="保存YUV420 .npy")
    ap.add_argument("--out", default=os.path.expanduser("~/ir_bench_results"))
    ap.add_argument("--frames-dir", default="/dev/shm/ir_frames")
    args = ap.parse_args()

    blocked = args.blocked_level
    offset_ns = int(args.offset_ms * 1e6)
    os.makedirs(args.out, exist_ok=True)
    os.makedirs(args.frames_dir, exist_ok=True)

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.out, f"capture_{stamp}.csv")
    fields = ["shot", "age_ms", "age_comp_ms", "waited_ms", "timeout",
              "candidates", "sensor_fps", "temp_c", "throttled", "file"]
    cf = open(csv_path, "w", newline="", encoding="utf-8")
    cw = csv.DictWriter(cf, fieldnames=fields)
    cw.writeheader()

    cam = Picamera2()
    controls = {}
    if args.fps > 0:
        d = int(1_000_000 / args.fps)
        controls["FrameDurationLimits"] = (d, d)
    cfg = cam.create_video_configuration(
        main={"size": SIZE, "format": "YUV420"}, buffer_count=4, controls=controls)
    cam.configure(cfg)
    cam.start()

    ring = None
    rows = []
    try:
        isp = {}
        if not args.no_lock:
            isp = lock_isp(cam)
            print(f"ISP 已锁定: {isp}")
        ring = Ring(cam, RING_N)
        ring.start()
        time.sleep(1.0)
        print(f"传感器实测帧率: {ring.sensor_fps():.1f} fps | offset={args.offset_ms}ms | "
              f"最大等待={args.max_wait_ms}ms")
        print(f"CSV: {csv_path}\n")

        shot = 0
        while shot < args.shots:
            while GPIO.input(PIN) == blocked:
                time.sleep(0.003)
            print(f"[{shot+1}/{args.shots}] 请遮挡 ...", flush=True)

            while True:
                while GPIO.input(PIN) != blocked:
                    time.sleep(0.0005)
                t0 = time.monotonic()
                t_ns = now_ns()
                ok = True
                while time.monotonic() - t0 < args.debounce_ms / 1000:
                    if GPIO.input(PIN) != blocked:
                        ok = False
                        break
                    time.sleep(0.0002)
                if ok:
                    break

            stale_before = (t_ns - ring.last_host_ns) / 1e6 if ring.last_host_ns else 1e9
            got = ring.wait_first_after(t_ns, offset_ns, args.max_wait_ms / 1000)
            waited_ms = (now_ns() - t_ns) / 1e6
            frames = ring.snapshot()
            stale_after = (now_ns() - ring.last_host_ns) / 1e6 if ring.last_host_ns else 1e9

            if not frames or stale_before > STALE_MS or stale_after > STALE_MS:
                print(f"    [无效] 采帧停滞 (before={stale_before:.0f}ms after={stale_after:.0f}ms), 请重试")
                while GPIO.input(PIN) == blocked:
                    time.sleep(0.003)
                time.sleep(0.3)
                continue

            bi = min(range(len(frames)),
                     key=lambda i: abs(frames[i][0] + offset_ns - t_ns))
            ts, host, arr = frames[bi]
            age = (t_ns - ts) / 1e6
            age_comp = (t_ns - (ts + offset_ns)) / 1e6

            fpath = ""
            if args.save_frames:
                fpath = os.path.join(args.frames_dir, f"{stamp}_{shot+1}.npy")
                np.save(fpath, arr)
                meta = {
                    "shot": shot + 1, "trigger_boottime_ns": t_ns,
                    "sensor_ts_ns": ts, "frame_age_ms": age,
                    "offset_ms": args.offset_ms, "isp": isp,
                    "sensor_fps": ring.sensor_fps(), "shape": list(arr.shape),
                }
                with open(fpath[:-4] + ".json", "w") as f:
                    json.dump(meta, f)

            row = {
                "shot": shot + 1, "age_ms": round(age, 2),
                "age_comp_ms": round(age_comp, 2), "waited_ms": round(waited_ms, 1),
                "timeout": 0 if got else 1, "candidates": len(frames),
                "sensor_fps": round(ring.sensor_fps(), 1),
                "temp_c": round(temp_c(), 1), "throttled": throttled(), "file": fpath,
            }
            cw.writerow(row)
            cf.flush()
            rows.append(row)
            shot += 1
            print(f"    帧龄 {age:+.1f}ms | 补偿后 {age_comp:+.1f}ms | 等待 {waited_ms:.0f}ms"
                  f"{' (超时)' if not got else ''} | {ring.sensor_fps():.1f}fps")

            while GPIO.input(PIN) == blocked:
                time.sleep(0.003)
            time.sleep(0.3)
            gc.collect()

    except KeyboardInterrupt:
        print("\n用户中断, 输出已完成部分汇总")
    finally:
        if ring:
            ring.stop()
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
    ages = [r["age_ms"] for r in rows]
    comp = [r["age_comp_ms"] for r in rows]
    waits = [r["waited_ms"] for r in rows]
    print("\n" + "=" * 60)
    print(f"样本数 {len(rows)} | 超时 {sum(r['timeout'] for r in rows)} 次")
    print(f"帧龄(未补偿) 带符号均值 {statistics.mean(ages):+.1f}ms  "
          f"|均值| {statistics.mean(abs(a) for a in ages):.1f}ms  最大 {max(abs(a) for a in ages):.1f}ms")
    print(f"帧龄(补偿后) 带符号均值 {statistics.mean(comp):+.1f}ms  "
          f"|均值| {statistics.mean(abs(a) for a in comp):.1f}ms  最大 {max(abs(a) for a in comp):.1f}ms")
    if len(ages) > 1:
        print(f"帧龄标准差 {statistics.stdev(ages):.1f}ms")
    print(f"等待时间 均值 {statistics.mean(waits):.0f}ms  最大 {max(waits):.0f}ms")
    print(f"建议 --offset-ms ≈ {-statistics.mean(ages):+.1f}  (用 offset=0 的数据时才有意义)")
    print(f"CSV: {csv_path}")


if __name__ == "__main__":
    main()
