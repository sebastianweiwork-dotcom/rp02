#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import shutil
import subprocess

OUT_FILE = "/tmp/csi_selftest.jpg"

def run(cmd, timeout=15):
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return p.returncode, p.stdout.strip(), p.stderr.strip()

def main():
    print("=== CSI 相机自检开始 ===")

    # 1) 检查命令
    if not shutil.which("rpicam-still"):
        print("[ERR ] 未找到 rpicam-still。请安装/启用 libcamera 工具链。")
        print("       可尝试: sudo apt install rpicam-apps")
        sys.exit(2)

    # 2) 列出相机
    print("\n[1/3] 检测相机 ...")
    rc, out, err = run(["rpicam-still", "--list-cameras"], timeout=10)
    txt = (out + "\n" + err).strip()
    print(txt if txt else "(无输出)")

    # 粗略判断是否发现相机
    if rc != 0 or ("Available cameras" not in txt and "0 : " not in txt and "1 : " not in txt):
        print("[ERR ] 未检测到可用 CSI 相机。")
        print("       请检查排线方向、接口是否插紧、是否启用了摄像头接口。")
        sys.exit(1)

    # 3) 拍照测试
    print("\n[2/3] 拍照测试 ...")
    if os.path.exists(OUT_FILE):
        os.remove(OUT_FILE)

    cmd = [
        "rpicam-still",
        "-o", OUT_FILE,
        "--timeout", "1200",   # 1.2秒预热
        "--nopreview"
    ]
    rc, out, err = run(cmd, timeout=20)
    if out:
        print(out)
    if err:
        print(err)

    if rc != 0 or not os.path.exists(OUT_FILE):
        print("[ERR ] 拍照失败，CSI 相机可能未正常工作。")
        sys.exit(1)

    # 4) 检查图片有效性（优先用 identify，没有就只看文件大小）
    print("\n[3/3] 验证图片 ...")
    size = os.path.getsize(OUT_FILE)
    print(f"[ OK ] 输出文件: {OUT_FILE} ({size} bytes)")

    if shutil.which("identify"):
        rc, out, err = run(["identify", OUT_FILE], timeout=5)
        if rc == 0 and out:
            print(f"[ OK ] 图片信息: {out}")
        else:
            print("[WARN] identify 无法解析图片，但文件已生成。")
    else:
        if size < 10_000:
            print("[WARN] 文件较小，可能异常，请人工查看图片。")

    print("\n✅ CSI 相机工作正常（已成功拍照）。")
    print(f"你可以查看: {OUT_FILE}")

if __name__ == "__main__":
    main()
