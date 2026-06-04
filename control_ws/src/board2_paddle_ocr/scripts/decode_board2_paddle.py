#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""识别板二离线测试：PaddleOCR HTTP（Python 2.7）。"""

from __future__ import print_function

import argparse
import os
import sys

from paddle_ocr_client import (
    DEFAULT_BASE_URL,
    board2_decode_http,
    health_check,
    print_utf8,
    write_json_utf8,
)


def _default_output_dir(image_path, snapshot_dir):
    stem = os.path.splitext(os.path.basename(image_path))[0]
    base = snapshot_dir or os.getcwd()
    return os.path.join(base, "output", "%s_board2_paddle" % stem)


def main(argv=None):
    parser = argparse.ArgumentParser(description="板二 PaddleOCR HTTP 离线识别")
    parser.add_argument("image", help="输入图片路径")
    parser.add_argument("-o", "--output", dest="output_dir", default=None)
    parser.add_argument(
        "--snapshot-dir",
        default=None,
        help="结果根目录，默认当前目录",
    )
    parser.add_argument("--url", default=None, help="Paddle OCR 服务 URL")
    parser.add_argument("--timeout", type=float, default=None)
    args = parser.parse_args(argv)

    image_path = os.path.abspath(args.image)
    if not os.path.isfile(image_path):
        print("FAIL: 图片不存在:", image_path)
        return 1

    base_url = args.url or DEFAULT_BASE_URL
    if not health_check(base_url):
        print("FAIL: Paddle OCR 服务未启动")
        print("     请运行: roscd board2_paddle_ocr && ./run_paddle_ocr_server.sh")
        print("     url:", base_url)
        return 1

    result = board2_decode_http(
        image_path,
        base_url=base_url,
        timeout=args.timeout,
    )

    output_dir = args.output_dir or _default_output_dir(
        image_path, args.snapshot_dir
    )
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)

    out = {
        "image_path": image_path,
        "output_dir": os.path.abspath(output_dir),
        "paddle_url": base_url,
        "ok": result["ok"],
        "error": result["error"],
        "raw_text": result["raw_text"],
        "speech_text": result["speech_text"],
        "wait_seconds": result["wait_seconds"],
    }
    result_path = os.path.join(output_dir, "result.json")
    write_json_utf8(result_path, out)

    print("output_dir:", out["output_dir"])
    print_utf8("speech_text:", out["speech_text"])
    print_utf8("raw_text:", out["raw_text"])
    print("wait_seconds:", out["wait_seconds"])
    if not result["ok"]:
        print_utf8("error:", result["error"])
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
