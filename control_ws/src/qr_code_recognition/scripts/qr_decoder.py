#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""识别板一：全图 pyzbar + 亮区质心分 slot（不依赖四黑框）。

相对亮区质心四象限：左上 1、右上 2、左下 3、右下 4。
成功条件：扫到 ≥1 个有效二维码即可，不要求四格齐全。
"""


import os

import cv2
import numpy as np
from pyzbar.pyzbar import decode as pyzbar_decode


class Board1DecodeParams(object):
    """板一解码参数（rosparam 可覆盖）。"""

    def __init__(
        self,
        bright_thresh=165,
        min_bright_pixels=30,
        min_qr_area=120.0,
        save_slot_crops=True,
        crop_margin_ratio=0.15,
    ):
        self.bright_thresh = int(bright_thresh)
        self.min_bright_pixels = int(min_bright_pixels)
        self.min_qr_area = float(min_qr_area)
        self.save_slot_crops = bool(save_slot_crops)
        self.crop_margin_ratio = float(crop_margin_ratio)

    @classmethod
    def from_rosparam(cls, node_handle=None):
        if node_handle is None:
            import rospy
            node_handle = rospy

        return cls(
            bright_thresh=node_handle.get_param("~bright_thresh", 165),
            min_bright_pixels=node_handle.get_param("~min_bright_pixels", 30),
            min_qr_area=node_handle.get_param("~min_qr_area", 120.0),
            save_slot_crops=node_handle.get_param("~save_slot_crops", True),
            crop_margin_ratio=node_handle.get_param("~crop_margin_ratio", 0.15),
        )


# 兼容旧 import 名
FrameDetectParams = Board1DecodeParams


def _ensure_dir(path):
    if not path or os.path.isdir(path):
        return
    try:
        os.makedirs(path)
    except OSError:
        if not os.path.isdir(path):
            raise


def _crop_paths_for_source(source_image_path):
    base, _ext = os.path.splitext(source_image_path)
    return {slot: "%s_slot%d.jpg" % (base, slot) for slot in (1, 2, 3, 4)}


def compute_bright_centroid(gray, bright_thresh=200, min_pixels=50):
    mask = gray >= int(bright_thresh)
    count = int(np.count_nonzero(mask))
    if count < min_pixels:
        return None, count

    ys, xs = np.where(mask)
    return (float(np.mean(xs)), float(np.mean(ys))), count


def assign_slot_from_centroid(qr_cx, qr_cy, ref_cx, ref_cy):
    col = 1 if qr_cx < ref_cx else 2
    row = 0 if qr_cy < ref_cy else 1
    return row * 2 + col


def _decode_pyzbar_objects(image_bgr, min_area=120):
    out = []
    for obj in pyzbar_decode(image_bgr):
        try:
            text = obj.data.decode("utf-8").strip()
        except (AttributeError, UnicodeDecodeError):
            try:
                text = str(obj.data).strip()
            except Exception:
                continue
        if not text:
            continue

        rect = obj.rect
        area = float(rect.width * rect.height)
        if area < min_area:
            continue

        cx = rect.left + rect.width / 2.0
        cy = rect.top + rect.height / 2.0
        out.append((text, cx, cy, area, rect))
    return out


def scan_qr_codes(image_bgr, min_area=120):
    found = {}
    h, w = image_bgr.shape[:2]
    for s in (1.0, 1.5, 2.0):
        if abs(s - 1.0) < 1e-3:
            work = image_bgr
        else:
            work = cv2.resize(
                image_bgr,
                (int(w * s), int(h * s)),
                interpolation=cv2.INTER_CUBIC,
            )
        inv_scale = 1.0 / s
        area_thresh = min_area * s * s
        for text, cx, cy, area, rect in _decode_pyzbar_objects(work, min_area=area_thresh):
            cx *= inv_scale
            cy *= inv_scale
            area *= inv_scale * inv_scale
            rect_left = rect.left * inv_scale
            rect_top = rect.top * inv_scale
            rect_w = rect.width * inv_scale
            rect_h = rect.height * inv_scale
            key = (text, int(cx / 8), int(cy / 8))
            if key not in found or area > found[key][3]:
                found[key] = (text, cx, cy, area, rect_left, rect_top, rect_w, rect_h)
    return list(found.values())


def _crop_qr_region(image, left, top, width, height, margin_ratio):
    h_img, w_img = image.shape[:2]
    mx = max(2, int(width * margin_ratio))
    my = max(2, int(height * margin_ratio))
    x1 = max(0, int(left) - mx)
    y1 = max(0, int(top) - my)
    x2 = min(w_img, int(left + width) + mx)
    y2 = min(h_img, int(top + height) + my)
    if x2 <= x1 or y2 <= y1:
        return None
    return image[y1:y2, x1:x2]


def decode_qr_by_bright_center(image_bgr, params=None):
    if params is None:
        params = Board1DecodeParams()

    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    bright_center, bright_count = compute_bright_centroid(
        gray,
        bright_thresh=params.bright_thresh,
        min_pixels=params.min_bright_pixels,
    )

    h, w = gray.shape[:2]
    if bright_center is None:
        ref_cx, ref_cy = w / 2.0, h / 2.0
    else:
        ref_cx, ref_cy = bright_center

    raw_hits = scan_qr_codes(image_bgr, min_area=params.min_qr_area)
    if not raw_hits:
        inv = cv2.bitwise_not(gray)
        inv_bgr = cv2.cvtColor(inv, cv2.COLOR_GRAY2BGR)
        raw_hits = scan_qr_codes(inv_bgr, min_area=params.min_qr_area)

    slot_buckets = {1: [], 2: [], 3: [], 4: []}
    for hit in raw_hits:
        text, cx, cy, area, left, top, rw, rh = hit
        slot = assign_slot_from_centroid(cx, cy, ref_cx, ref_cy)
        slot_buckets[slot].append(
            {
                "text": text,
                "center_x": cx,
                "center_y": cy,
                "area": area,
                "slot": slot,
                "rect": (left, top, rw, rh),
            }
        )

    qr_list = []
    for slot in (1, 2, 3, 4):
        items = slot_buckets[slot]
        if not items:
            continue
        best = max(items, key=lambda it: it["area"])
        qr_list.append(
            {
                "text": best["text"],
                "center_x": best["center_x"],
                "center_y": best["center_y"],
                "slot": slot,
                "_rect": best["rect"],
            }
        )

    meta = {
        "bright_center": (ref_cx, ref_cy),
        "bright_pixel_count": bright_count,
        "raw_decode_count": len(raw_hits),
    }
    return qr_list, meta


class QrDecodeResult(object):
    __slots__ = ("qr_list", "error_message", "frames_detected", "meta")

    def __init__(
        self,
        qr_list=None,
        error_message="",
        frames_detected=False,
        meta=None,
    ):
        self.qr_list = qr_list or []
        self.error_message = error_message
        self.frames_detected = frames_detected
        self.meta = meta or {}


def decode_qr(image, source_image_path=None, params=None):
    if params is None:
        params = Board1DecodeParams()

    crop_paths = None
    if source_image_path:
        crop_paths = _crop_paths_for_source(source_image_path)
        _ensure_dir(os.path.dirname(source_image_path) or ".")

    if image is None or image.size == 0:
        return QrDecodeResult(error_message="image_empty: 输入图像为空")

    qr_list, meta = decode_qr_by_bright_center(image, params=params)

    if not qr_list:
        return QrDecodeResult(
            error_message="decode_failed: 全图未扫到任何二维码（扫到 1 个即可，不要求四格齐全）",
            frames_detected=False,
            meta=meta,
        )

    if crop_paths and params.save_slot_crops:
        for qr in qr_list:
            slot = qr["slot"]
            rect = qr.get("_rect")
            if rect is None:
                continue
            crop = _crop_qr_region(
                image,
                rect[0],
                rect[1],
                rect[2],
                rect[3],
                params.crop_margin_ratio,
            )
            if crop is not None:
                cv2.imwrite(crop_paths[slot], crop)

    clean_list = []
    for qr in qr_list:
        clean_list.append(
            {
                "text": qr["text"],
                "center_x": qr["center_x"],
                "center_y": qr["center_y"],
                "slot": qr["slot"],
            }
        )

    return QrDecodeResult(
        qr_list=clean_list,
        frames_detected=True,
        meta=meta,
    )
