#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Python 2.7 调用 PaddleOCR HTTP 服务（识别板二）。"""

from __future__ import print_function

import argparse
import io
import json
import os
import sys

try:
    text_type = unicode
except NameError:
    text_type = str

DEFAULT_BASE_URL = os.environ.get("PADDLE_OCR_URL", "http://127.0.0.1:8765")
DEFAULT_TIMEOUT = float(os.environ.get("PADDLE_OCR_TIMEOUT", "120"))


def _to_text(value):
    if value is None:
        return ""
    if isinstance(value, text_type):
        return value
    if isinstance(value, str):
        return value.decode("utf-8", "ignore")
    if isinstance(value, bytes):
        return value.decode("utf-8", "ignore")
    return text_type(value)


def write_json_utf8(path, data):
    """写入含中文的 JSON（兼容 Python 2.7 / 3）。"""
    text = json.dumps(data, ensure_ascii=False, indent=2)
    if sys.version_info[0] < 3:
        if isinstance(text, text_type):
            blob = text.encode("utf-8")
        else:
            blob = text
        with open(path, "wb") as fp:
            fp.write(blob)
    else:
        if isinstance(text, bytes):
            text = text.decode("utf-8")
        with io.open(path, "w", encoding="utf-8") as fp:
            fp.write(text)


def print_utf8(label, value):
    text = _to_text(value)
    if sys.version_info[0] < 3 and isinstance(text, text_type):
        sys.stdout.write("%s %s\n" % (label, text.encode("utf-8")))
    else:
        print("%s %s" % (label, text))


def _http_post_json(url, payload, timeout):
    body = json.dumps(payload)
    if sys.version_info[0] < 3:
        if isinstance(body, text_type):
            body = body.encode("utf-8")

    import urllib2

    req = urllib2.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json"},
    )
    try:
        resp = urllib2.urlopen(req, timeout=timeout)
        raw = resp.read()
        status = resp.getcode()
    except urllib2.HTTPError as exc:
        status = exc.code
        raw = exc.read()
    except Exception as exc:
        return 0, {
            "ok": False,
            "error": "http_failed: %s" % exc,
            "raw_text": "",
            "speech_text": "",
            "wait_seconds": 0,
        }

    if sys.version_info[0] < 3 and isinstance(raw, str):
        raw = raw.decode("utf-8", "ignore")

    try:
        data = json.loads(raw)
    except ValueError:
        data = {
            "ok": False,
            "error": "invalid_json_response",
            "raw_text": "",
            "speech_text": "",
            "wait_seconds": 0,
        }
    return status, data


def board2_decode_http(
    image_path,
    base_url=None,
    timeout=None,
    use_keyword_inference=True,
):
    base_url = (base_url or DEFAULT_BASE_URL).rstrip("/")
    timeout = timeout if timeout is not None else DEFAULT_TIMEOUT
    image_path = os.path.abspath(image_path)

    if not os.path.isfile(image_path):
        return {
            "ok": False,
            "error": "image_not_found: %s" % image_path,
            "raw_text": "",
            "speech_text": "",
            "wait_seconds": 0,
        }

    url = base_url + "/ocr/board2"
    payload = {
        "image_path": image_path,
        "use_keyword_inference": use_keyword_inference,
    }
    status, data = _http_post_json(url, payload, timeout)

    if not isinstance(data, dict):
        data = {}

    result = {
        "ok": bool(data.get("ok")),
        "error": _to_text(data.get("error", "")),
        "raw_text": _to_text(data.get("raw_text", "")),
        "speech_text": _to_text(data.get("speech_text", "")),
        "wait_seconds": int(data.get("wait_seconds", 0) or 0),
        "http_status": status,
    }
    if status and status >= 400 and not result["error"]:
        result["error"] = "http_status_%d" % status
        result["ok"] = False
    return result


def health_check(base_url=None, timeout=5):
    base_url = (base_url or DEFAULT_BASE_URL).rstrip("/")
    try:
        import urllib2

        resp = urllib2.urlopen(base_url + "/health", timeout=timeout)
        raw = resp.read()
        if sys.version_info[0] < 3 and isinstance(raw, str):
            raw = raw.decode("utf-8", "ignore")
        data = json.loads(raw)
        return bool(data.get("ok"))
    except Exception:
        return False


def main(argv=None):
    parser = argparse.ArgumentParser(description="Python2 调用 PaddleOCR HTTP 服务")
    parser.add_argument("image", nargs="?", help="图片路径")
    parser.add_argument("--url", default=DEFAULT_BASE_URL, help="服务根 URL")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    parser.add_argument("--health", action="store_true", help="仅检查服务是否在线")
    parser.add_argument(
        "--no-keyword-inference",
        dest="use_keyword_inference",
        action="store_false",
        default=True,
    )
    args = parser.parse_args(argv)

    if args.health:
        ok = health_check(args.url, timeout=min(args.timeout, 10))
        print("health:", ok, "url:", args.url)
        return 0 if ok else 1

    if not args.image:
        parser.error("请提供 image 或使用 --health")

    result = board2_decode_http(
        args.image,
        base_url=args.url,
        timeout=args.timeout,
        use_keyword_inference=args.use_keyword_inference,
    )

    print("ok:", result["ok"])
    if result["error"]:
        print_utf8("error:", result["error"])
    print_utf8("speech_text:", result["speech_text"])
    print_utf8("raw_text:", result["raw_text"])
    print("wait_seconds:", result["wait_seconds"])
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
