#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import cv2
import time
import signal
import psutil
import traceback
import gc
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
CAMERA_BUFFER_COUNT = 3          # libcamera内部缓冲数，越小越省内存

# 关键点/ROI 参数
X = 960                          # ROI中心x
Y = 540                          # ROI中心y
ROI_N = 9                        # ROI正方形边长

# Hue触发范围（OpenCV HSV: H范围0-179）
HUE_MIN = 159
HUE_MAX = 171

# 冷却时间（秒）
COOLDOWN_SEC = 3.0

# 图片缓存队列大小（帧）
# 1080p YUV420约3MB/帧；20帧约60MB（Zero 2 W更稳）
FRAME_BUFFER_SIZE = 20

# 输出目录
OUTPUT_ROOT = "/home/rp02/hsv trigger 2 test"

# 终端状态输出间隔（秒）
STATUS_INTERVAL_SEC = 1.0

# JPG保存质量（适当降低可减小写盘压力）
JPEG_QUALITY = 85

# CSV日志批量落盘（避免hsv日志无限占内存）
CSV_FLUSH_EVERY = 300

# 触发保存时是否暂停相机（强烈建议True，降低二次触发时崩溃概率）
STOP_CAMERA_DURING_SAVE = True


RUNNING = True


def handle_sigint(sig, frame):
    global RUNNING
    RUNNING = False


def now_str_ms():
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]


def get_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return None


def get_system_info():
    mem = psutil.virtual_memory()
    return {
        "cpu_percent": psutil.cpu_percent(interval=None),
        "mem_percent": mem.percent,
        "mem_used_mb": mem.used / (1024 * 1024),
        "mem_total_mb": mem.total / (1024 * 1024),
        "cpu_temp": get_cpu_temp(),
    }


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def roi_hsv_mean(yuv, width, height, x0, y0, x1, y1):
    """
    仅对ROI做 YUV420(I420) -> BGR -> HSV，返回H/S/V均值
    I420布局:
      - Y平面: [0:height, 0:width]
      - U平面: [height:height+height/4, :] reshape成(height/2, width/2)
      - V平面: [height+height/4:height+height/2, :] reshape成(height/2, width/2)
    """
    y_plane = yuv[0:height, 0:width]
    u_plane = yuv[height:height + height // 4, :].reshape(height // 2, width // 2)
    v_plane = yuv[height + height // 4:height + height // 2, :].reshape(height // 2, width // 2)

    ys = y_plane[y0:y1, x0:x1].astype(np.float32)

    # 色度平面是2x2下采样，映射ROI到U/V索引
    ux = np.arange(x0, x1) // 2
    uy = np.arange(y0, y1) // 2
    us = u_plane[np.ix_(uy, ux)].astype(np.float32) - 128.0
    vs = v_plane[np.ix_(uy, ux)].astype(np.float32) - 128.0

    # BT.601近似（与OpenCV I420转换接近）
    ys = (ys - 16.0) * 1.164
    r = ys + 1.596 * vs
    g = ys - 0.392 * us - 0.813 * vs
    b = ys + 2.017 * us

    bgr = np.stack([b, g, r], axis=-1)
    bgr = np.clip(bgr, 0, 255).astype(np.uint8)

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return (
        float(hsv[:, :, 0].mean()),
        float(hsv[:, :, 1].mean()),
        float(hsv[:, :, 2].mean())
    )


def start_camera(picam2, width, height, fps):
    config = picam2.create_video_configuration(
        main={"size": (width, height), "format": "YUV420"},
        controls={"FrameDurationLimits": (int(1e6 / fps), int(1e6 / fps))},
        buffer_count=CAMERA_BUFFER_COUNT,
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.2)


def main():
    global RUNNING
    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    width, height, fps = WIDTH, HEIGHT, FPS
    x, y = X, Y
    roi_n = max(1, ROI_N)
    hue_min = clamp(HUE_MIN, 0, 179)
    hue_max = clamp(HUE_MAX, 0, 179)
    cooldown_sec = max(0.0, COOLDOWN_SEC)
    frame_buffer_size = max(1, FRAME_BUFFER_SIZE)

    # 固定ROI边界（参数固定时只算一次）
    half = roi_n // 2
    x0 = clamp(x - half, 0, width - 1)
    y0 = clamp(y - half, 0, height - 1)
    x1 = clamp(x0 + roi_n, 1, width)
    y1 = clamp(y0 + roi_n, 1, height)

    session_dir = Path(OUTPUT_ROOT) / now_str_ms()
    session_dir.mkdir(parents=True, exist_ok=True)
    csv_path = session_dir / "hsv_log.csv"

    print(f"[INFO] session_dir: {session_dir}")
    print(f"[INFO] csv_path   : {csv_path}")

    # 图像缓存队列（滚动）
    frame_queue = deque(maxlen=frame_buffer_size)

    # HSV日志内存缓冲（小批量，定期刷盘）
    hsv_buffer = []
    fieldnames = [
        "frame_id", "timestamp_unix", "timestamp_iso",
        "x", "y", "roi_n",
        "h_mean", "s_mean", "v_mean", "triggered"
    ]
    csv_file = open(csv_path, "w", newline="", encoding="utf-8")
    csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    csv_writer.writeheader()

    picam2 = Picamera2()
    start_camera(picam2, width, height, fps)

    print("[INFO] Camera started.")
    print(f"[INFO] Capture: {width}x{height} @ {fps}fps, format=YUV420")
    print(f"[INFO] ROI center=({x}, {y}), roi_n={roi_n}")
    print(f"[INFO] Hue trigger range=[{hue_min}, {hue_max}]")
    print(f"[INFO] Frame buffer size={frame_buffer_size}")
    print(f"[INFO] Cooldown={cooldown_sec}s")
    print(f"[INFO] CSV flush every={CSV_FLUSH_EVERY} rows")
    print(f"[INFO] Stop camera during save={STOP_CAMERA_DURING_SAVE}")
    print("[INFO] Press Ctrl+C to stop.")

    frame_id = 0
    last_trigger_time = 0.0
    last_status_print = 0.0

    try:
        while RUNNING:
            yuv = picam2.capture_array("main")
            t_now = time.time()
            frame_id += 1

            # 放入滚动缓存（深拷贝，防止底层复用）
            frame_queue.append({
                "frame_id": frame_id,
                "t_unix": t_now,
                "yuv420": np.array(yuv, copy=True),
            })

            # 仅ROI做HSV
            h_mean, s_mean, v_mean = roi_hsv_mean(yuv, width, height, x0, y0, x1, y1)

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

            in_hue_range = (hue_min <= h_mean <= hue_max)
            can_trigger = ((t_now - last_trigger_time) >= cooldown_sec)

            if in_hue_range and can_trigger:
                trigger_dir = session_dir / f"trigger_{now_str_ms()}"
                trigger_dir.mkdir(parents=True, exist_ok=True)

                pending = len(frame_queue)
                print(f"\n[TRIGGER] frame_id={frame_id}, h_mean={h_mean:.2f}, "
                      f"saving {pending} frames -> {trigger_dir}")

                # 触发前刷一次CSV，减少异常中断时数据丢失
                if hsv_buffer:
                    csv_writer.writerows(hsv_buffer)
                    csv_file.flush()
                    hsv_buffer.clear()

                gc.collect()

                # 保存期间暂停相机，降低内存和总线压力
                if STOP_CAMERA_DURING_SAVE:
                    try:
                        picam2.stop()
                        time.sleep(0.1)
                    except Exception:
                        pass

                saved = 0
                # 边保存边释放，避免二次内存峰值
                while frame_queue:
                    item = frame_queue.popleft()
                    bgr_f = cv2.cvtColor(item["yuv420"], cv2.COLOR_YUV2BGR_I420)
                    t_ms = int(item["t_unix"] * 1000)
                    jpg_path = trigger_dir / f"{saved + 1:06d}_{t_ms}.jpg"
                    cv2.imwrite(str(jpg_path), bgr_f, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])

                    saved += 1
                    del bgr_f, item

                gc.collect()

                if STOP_CAMERA_DURING_SAVE:
                    try:
                        picam2.start()
                        time.sleep(0.2)
                    except Exception:
                        # 如果start失败，尝试重配并启动
                        try:
                            start_camera(picam2, width, height, fps)
                        except Exception:
                            print("[ERROR] Failed to restart camera after trigger save.")
                            raise

                hsv_item["triggered"] = 1
                last_trigger_time = time.time()  # 从保存结束开始计冷却
                print(f"[TRIGGER] done. saved={saved}. buffer cleared. cooldown {cooldown_sec}s\n")

            hsv_buffer.append(hsv_item)

            # 定期写CSV，防止内存累计
            if len(hsv_buffer) >= CSV_FLUSH_EVERY:
                csv_writer.writerows(hsv_buffer)
                csv_file.flush()
                hsv_buffer.clear()

            # 状态输出
            if (t_now - last_status_print) >= STATUS_INTERVAL_SEC:
                s = get_system_info()
                temp_txt = f"{s['cpu_temp']:.1f}C" if s["cpu_temp"] is not None else "N/A"
                print(
                    f"[STATUS] frame={frame_id} "
                    f"H={h_mean:.2f} S={s_mean:.2f} V={v_mean:.2f} "
                    f"buffer={len(frame_queue)}/{frame_buffer_size} "
                    f"CPU={s['cpu_percent']:.1f}% "
                    f"MEM={s['mem_percent']:.1f}%({s['mem_used_mb']:.0f}/{s['mem_total_mb']:.0f}MB) "
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

        print(f"[INFO] Writing remaining CSV rows: {csv_path}")
        try:
            if hsv_buffer:
                csv_writer.writerows(hsv_buffer)
                hsv_buffer.clear()
            csv_file.flush()
            csv_file.close()
        except Exception:
            pass

        print("[INFO] Done.")


if __name__ == "__main__":
    main()
