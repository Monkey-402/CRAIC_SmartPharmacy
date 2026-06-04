# -*- coding: utf-8 -*-
"""PaddleOCR HTTP 服务（Python 3，在 conda 环境 paddleocr 中运行）。

启动:
  roscd board2_paddle_ocr && ./run_paddle_ocr_server.sh

接口:
  GET  /health
  POST /ocr/board2  JSON: {"image_path": "/path/to.jpg"}
                    或   {"image_base64": "<base64>"}
"""

from __future__ import print_function

import base64
import os
import sys
import tempfile

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from board2_postprocess import board2_result_from_raw  # noqa: E402

try:
    from flask import Flask, jsonify, request
except ImportError:
    print("缺少 flask，请在 conda 环境 paddleocr 中: pip install flask", file=sys.stderr)
    sys.exit(1)

app = Flask(__name__)
_ocr_engine = None


def _get_ocr():
    global _ocr_engine
    if _ocr_engine is None:
        print("[paddle_ocr_server] 正在加载 PaddleOCR 模型（首次较慢）...")
        from paddleocr import PaddleOCR

        use_gpu = os.environ.get("PADDLE_OCR_USE_GPU", "0") == "1"
        _ocr_engine = PaddleOCR(
            use_angle_cls=False,
            lang="ch",
            use_gpu=use_gpu,
            show_log=False,
        )
        print("[paddle_ocr_server] PaddleOCR 就绪")
    return _ocr_engine


def _paddle_lines_to_text(ocr_result):
    if not ocr_result:
        return ""
    lines = []
    block = ocr_result[0] if ocr_result else None
    if not block:
        return ""
    for item in block:
        if not item or len(item) < 2:
            continue
        text_part = item[1]
        if isinstance(text_part, (list, tuple)) and text_part:
            lines.append(text_part[0])
        elif isinstance(text_part, str):
            lines.append(text_part)
    return "\n".join(lines)


def _run_ocr_on_path(image_path):
    ocr = _get_ocr()
    result = ocr.ocr(image_path, cls=False)
    return _paddle_lines_to_text(result)


def _decode_image_bytes(data):
    try:
        import cv2
        import numpy as np
    except ImportError:
        raise RuntimeError("需要 opencv-python: pip install opencv-python-headless")

    arr = np.frombuffer(data, dtype=np.uint8)
    image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("无法解码图片数据")
    fd, path = tempfile.mkstemp(suffix=".jpg", prefix="paddle_ocr_")
    os.close(fd)
    cv2.imwrite(path, image)
    return path


@app.route("/health", methods=["GET"])
def health():
    return jsonify({"ok": True, "service": "paddle_ocr_board2"})


@app.route("/ocr/board2", methods=["POST"])
def ocr_board2():
    payload = request.get_json(silent=True) or {}
    use_keyword = payload.get("use_keyword_inference", True)
    temp_path = None

    try:
        image_path = payload.get("image_path")
        image_b64 = payload.get("image_base64")

        if image_path:
            image_path = os.path.abspath(image_path)
            if not os.path.isfile(image_path):
                return jsonify(
                    {
                        "ok": False,
                        "error": "image_not_found: %s" % image_path,
                        "raw_text": "",
                        "speech_text": "",
                        "wait_seconds": 0,
                    }
                ), 404
            raw_text = _run_ocr_on_path(image_path)
        elif image_b64:
            data = base64.b64decode(image_b64)
            temp_path = _decode_image_bytes(data)
            raw_text = _run_ocr_on_path(temp_path)
        else:
            return jsonify(
                {
                    "ok": False,
                    "error": "need image_path or image_base64",
                    "raw_text": "",
                    "speech_text": "",
                    "wait_seconds": 0,
                }
            ), 400

        out = board2_result_from_raw(raw_text, use_keyword_inference=use_keyword)
        out["ok"] = True
        out["error"] = ""
        return jsonify(out)

    except Exception as exc:
        return jsonify(
            {
                "ok": False,
                "error": str(exc),
                "raw_text": "",
                "speech_text": "",
                "wait_seconds": 0,
            }
        ), 500
    finally:
        if temp_path and os.path.isfile(temp_path):
            try:
                os.remove(temp_path)
            except OSError:
                pass


def main():
    host = os.environ.get("PADDLE_OCR_HOST", "127.0.0.1")
    port = int(os.environ.get("PADDLE_OCR_PORT", "8765"))
    debug = os.environ.get("PADDLE_OCR_DEBUG", "0") == "1"
    print("[paddle_ocr_server] http://%s:%d" % (host, port))
    app.run(host=host, port=port, debug=debug, threaded=False)


if __name__ == "__main__":
    main()
