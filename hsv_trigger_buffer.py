#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import cv2
import time
import signal
import psutil
import traceback
from pathlib import Path
from datetime import datetime
from collections import deque

import numpy as np
from picamera2 import Picamera2

# =========================
# 全部参数（集中在最前面）
# =========================

# 相机参数
WIDTH = 1920
HEIGHT = 1080
FPS = 30

# 关键点/ROI 参数
X = 960                  # ROI中心x
Y = 540                  # ROI中心y
ROI_N = 9                # ROI正方形边长（建议奇数）

# Hue触发范围（OpenCV HSV: H范围0-179）
HUE_MIN = 159
HUE_MAX = 171

# 冷却时间（秒）
COOLDOWN_SEC = 3.0

# 图片缓存队列大小（帧）
# 1080p YUV420约3MB/帧；90帧约270MB
FRAME_BUFFER_SIZE = 90

# 输出目录
OUTPUT_ROOT = "/home/rp02/hsv trigger 2 test"

# 终端状态输出间隔（秒）
STATUS_INTERVAL_SEC = 1.0

# JPG保存质量
JPEG_QUALITY = 95


RUNNING = True


def handle_sigint(sig, frame):
    global RUNNING
    RUNNING = False


def now_str_ms():
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def get_cpu_temp():
    thermal = "/sys/class/thermal/thermal_zone0/temp"
    try:
        with open(thermal, "r") as f:
            milli = int(f.read().strip())
            return milli / 1000.0
    except Exception:
        return None


def get_system_info():
    cpu_percent = psutil.cpu_percent(interval=None)
    mem = psutil.virtual_memory()
    temp = get_cpu_temp()
    return {
        "cpu_percent": cpu_percent,
        "mem_percent": mem.percent,
        "mem_used_mb": mem.used / (1024 * 1024),
        "mem_total_mb": mem.total / (1024 * 1024),
        "cpu_temp": temp
    }


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    global RUNNING
    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    width = WIDTH
    height = HEIGHT
    fps = FPS

    x = X
    y = Y
    roi_n = max(1, ROI_N)
    hue_min = clamp(HUE_MIN, 0, 179)
    hue_max = clamp(HUE_MAX, 0, 179)
    cooldown_sec = max(0.0, COOLDOWN_SEC)
    frame_buffer_size = max(1, FRAME_BUFFER_SIZE)

    session_dir = Path(OUTPUT_ROOT) / now_str_ms()
    ensure_dir(session_dir)

    csv_path = session_dir / "hsv_log.csv"
    print(f"[INFO] session_dir: {session_dir}")
    print(f"[INFO] csv_path   : {csv_path}")

    frame_queue = deque(maxlen=frame_buffer_size)
    hsv_queue = []

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (width, height), "format": "YUV420"},
        controls={"FrameDurationLimits": (int(1e6 / fps), int(1e6 / fps))}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.2)

    print("[INFO] Camera started.")
    print(f"[INFO] Capture: {width}x{height} @ {fps}fps, format=YUV420")
    print(f"[INFO] ROI center=({x}, {y}), roi_n={roi_n}")
    print(f"[INFO] Hue trigger range=[{hue_min}, {hue_max}]")
    print(f"[INFO] Frame buffer size={frame_buffer_size}")
    print(f"[INFO] Cooldown={cooldown_sec}s")
    print("[INFO] Press Ctrl+C to stop.")

    frame_id = 0
    last_trigger_time = 0.0
    last_status_print = 0.0

    try:
        while RUNNING:
            t_now = time.time()

            yuv420 = picam2.capture_array("main")
            yuv_copy = np.array(yuv420, copy=True)

            frame_id += 1
            frame_queue.append({
                "frame_id": frame_id,
                "t_unix": t_now,
                "yuv420": yuv_copy
            })

            bgr = cv2.cvtColor(yuv_copy, cv2.COLOR_YUV2BGR_I420)
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

            half = roi_n // 2
            x0 = clamp(x - half, 0, width - 1)
            y0 = clamp(y - half, 0, height - 1)
            x1 = clamp(x0 + roi_n, 1, width)
            y1 = clamp(y0 + roi_n, 1, height)

            roi = hsv[y0:y1, x0:x1]
            if roi.size == 0:
                h_mean = s_mean = v_mean = 0.0
            else:
                h_mean = float(np.mean(roi[:, :, 0]))
                s_mean = float(np.mean(roi[:, :, 1]))
                v_mean = float(np.mean(roi[:, :, 2]))

            hsv_item = {
                "frame_id": frame_id,
                "timestamp_unix": t_now,
                "timestamp_iso": datetime.fromtimestamp(t_now).isoformat(timespec="milliseconds"),
                "x": x,
                "y": y,
                "roi_n": roi_n,
                "h_mean": h_mean,
                "s_mean": s_mean,
                "v_mean": v_mean,
                "triggered": 0
            }

            in_hue_range = (h_mean >= hue_min and h_mean <= hue_max)
            can_trigger = (t_now - last_trigger_time >= cooldown_sec)

            if in_hue_range and can_trigger:
                trigger_ts = now_str_ms()
                trigger_dir = session_dir / f"trigger_{trigger_ts}"
                ensure_dir(trigger_dir)

                print(f"\n[TRIGGER] frame_id={frame_id}, h_mean={h_mean:.2f}, saving {len(frame_queue)} buffered frames -> {trigger_dir}")

                for idx, item in enumerate(frame_queue, start=1):
                    yuv_f = item["yuv420"]
                    bgr_f = cv2.cvtColor(yuv_f, cv2.COLOR_YUV2BGR_I420)
                    t_ms = int(item["t_unix"] * 1000)
                    jpg_name = f"{idx:06d}_{t_ms}.jpg"
                    jpg_path = trigger_dir / jpg_name
                    cv2.imwrite(str(jpg_path), bgr_f, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])

                hsv_item["triggered"] = 1
                frame_queue.clear()
                last_trigger_time = t_now
                print(f"[TRIGGER] done. buffer cleared. cooldown {cooldown_sec}s\n")

            hsv_queue.append(hsv_item)

            if (t_now - last_status_print) >= STATUS_INTERVAL_SEC:
                sysi = get_system_info()
                temp_txt = f"{sysi['cpu_temp']:.1f}C" if sysi["cpu_temp"] is not None else "N/A"
                print(
                    f"[STATUS] frame={frame_id} "
                    f"H={h_mean:.2f} S={s_mean:.2f} V={v_mean:.2f} "
                    f"buffer={len(frame_queue)}/{frame_buffer_size} "
                    f"CPU={sysi['cpu_percent']:.1f}% "
                    f"MEM={sysi['mem_percent']:.1f}%({sysi['mem_used_mb']:.0f}/{sysi['mem_total_mb']:.0f}MB) "
                    f"TEMP={temp_txt}"
                )
                last_status_print = t_now

    except Exception as e:
        print("[ERROR] Exception in main loop:", e)
        traceback.print_exc()

    finally:
        print("\n[INFO] Stopping camera...")
        try:
            picam2.stop()
        except Exception:
            pass

        print(f"[INFO] Writing CSV log: {csv_path}")
        fieldnames = [
            "frame_id", "timestamp_unix", "timestamp_iso",
            "x", "y", "roi_n",
            "h_mean", "s_mean", "v_mean", "triggered"
        ]
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(hsv_queue)

        print("[INFO] Done.")


if __name__ == "__main__":
    main()
