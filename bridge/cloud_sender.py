#!/usr/bin/env python3
"""
cloud_sender.py — 触发 esp32_bridge 发送点云

用法:
  python3 cloud_sender.py                           # 自动查找 PCD
  python3 cloud_sender.py --pcd /path/to/scans.pcd  # 指定 PCD 文件

等效于: echo <path_or_auto> > ~/esp32_bridge/.cloud_trigger
"""

import argparse
import os
import sys

TRIGGER_FILE = os.path.expanduser("~/esp32_bridge/.cloud_trigger")


def main():
    ap = argparse.ArgumentParser(description="触发点云传输")
    ap.add_argument('--pcd', help='PCD 文件路径 (默认自动查找)')
    args = ap.parse_args()

    content = args.pcd if args.pcd else 'auto'
    if args.pcd and not os.path.isfile(args.pcd):
        print(f"错误: 文件不存在: {args.pcd}")
        sys.exit(1)

    with open(TRIGGER_FILE, 'w') as f:
        f.write(content)
    print(f"已触发点云传输: {content}")
    print(f"查看 esp32_bridge 终端了解发送进度")


if __name__ == '__main__':
    main()
