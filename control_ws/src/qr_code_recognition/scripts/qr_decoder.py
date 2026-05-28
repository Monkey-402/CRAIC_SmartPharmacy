#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""识别板一：检测四个黑框 → 分格裁剪 → 每格扫码。"""

import os

import cv2
from pyzbar.pyzbar import decode as pyzbar_decode


def _find_contours(binary):
    result = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(result) == 3:
        _image, contours, _hierarchy = result
    else:
        contours, _hierarchy = result
    return contours


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


def _crop_inset(image, x, y, w, h, margin_ratio=0.08):
    mx = max(2, int(w * margin_ratio))
    my = max(2, int(h * margin_ratio))
    x1 = max(0, x + mx)
    y1 = max(0, y + my)
    x2 = min(image.shape[1], x + w - mx)
    y2 = min(image.shape[0], y + h - my)
    if x2 <= x1 or y2 <= y1:
        return image[y : y + h, x : x + w]
    return image[y1:y2, x1:x2]


def _sort_frames_to_slots(rects):
    if len(rects) != 4:
        return None

    centers = []
    for x, y, w, h in rects:
        centers.append((x + w / 2.0, y + h / 2.0, x, y, w, h))

    mean_y = sum(c[1] for c in centers) / 4.0
    top = [c for c in centers if c[1] < mean_y]
    bottom = [c for c in centers if c[1] >= mean_y]

    if len(top) != 2 or len(bottom) != 2:
        centers.sort(key=lambda c: (c[1], c[0]))
        ordered = centers
    else:
        top.sort(key=lambda c: c[0])
        bottom.sort(key=lambda c: c[0])
        ordered = [top[0], top[1], bottom[0], bottom[1]]

    return [(int(r[2]), int(r[3]), int(r[4]), int(r[5])) for r in ordered]


def _is_squareish(w, h):
    if h <= 0:
        return False
    ratio = w / float(h)
    return 0.72 <= ratio <= 1.38


def _rect_from_contour(cnt):
    peri = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)
    if len(approx) == 4 and cv2.isContourConvex(approx):
        x, y, w, h = cv2.boundingRect(approx)
    else:
        x, y, w, h = cv2.boundingRect(cnt)

    if w < 40 or h < 40 or not _is_squareish(w, h):
        return None
    return (x, y, w, h)


def _collect_frame_rects(contours, img_area):
    rects = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < img_area * 0.025 or area > img_area * 0.42:
            continue
        rect = _rect_from_contour(cnt)
        if rect is not None:
            rects.append(rect)
    return rects


def _dedupe_rects(rects, dist_thresh=25):
    rects = sorted(rects, key=lambda r: r[2] * r[3], reverse=True)
    kept = []
    for rect in rects:
        cx = rect[0] + rect[2] / 2.0
        cy = rect[1] + rect[3] / 2.0
        for kx, ky, kw, kh in kept:
            kcx = kx + kw / 2.0
            kcy = ky + kh / 2.0
            if abs(cx - kcx) < dist_thresh and abs(cy - kcy) < dist_thresh:
                break
        else:
            kept.append(rect)
    return kept


def _pick_four_frame_rects(rects, img_area):
    rects = _dedupe_rects(rects)
    if len(rects) < 4:
        return None

    inner = [r for r in rects if r[2] * r[3] < img_area * 0.36]
    if len(inner) >= 4:
        rects = inner

    if len(rects) == 4:
        return _sort_frames_to_slots(rects)

    rects = sorted(rects, key=lambda r: r[2] * r[3], reverse=True)
    ref_area = rects[0][2] * rects[0][3]
    similar = [
        r for r in rects
        if ref_area * 0.45 <= (r[2] * r[3]) <= ref_area * 2.2
    ]
    if len(similar) < 4:
        return None

    similar = sorted(similar, key=lambda r: r[2] * r[3], reverse=True)[:4]
    return _sort_frames_to_slots(similar)


def detect_four_frames(image):
    """黑框轮廓检测四个窗口，顺序为 slot 1–4；失败返回 None。"""
    if image is None or image.size == 0:
        return None

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, inv = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    img_area = float(gray.shape[0] * gray.shape[1])
    kernels = (
        cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
        cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9)),
        cv2.getStructuringElement(cv2.MORPH_RECT, (13, 13)),
    )

    for k in kernels:
        mask = cv2.morphologyEx(inv, cv2.MORPH_CLOSE, k, iterations=2)
        contours = _find_contours(mask)
        rects = _collect_frame_rects(contours, img_area)
        picked = _pick_four_frame_rects(rects, img_area)
        if picked is not None:
            return picked

    contours = _find_contours(inv)
    rects = _collect_frame_rects(contours, img_area)
    return _pick_four_frame_rects(rects, img_area)


def _decode_text_from_crop(crop):
    try:
        results = pyzbar_decode(crop)
    except Exception:
        return None

    for r in results:
        try:
            return r.data.decode("utf-8")
        except (AttributeError, UnicodeDecodeError):
            try:
                return str(r.data)
            except Exception:
                continue
    return None


def decode_qr(image, source_image_path=None):
    crop_paths = None
    if source_image_path:
        crop_paths = _crop_paths_for_source(source_image_path)
        _ensure_dir(os.path.dirname(source_image_path))

    frames = detect_four_frames(image)
    if frames is None:
        return []

    qr_list = []
    for slot_index, (x, y, w, h) in enumerate(frames, start=1):
        crop = _crop_inset(image, x, y, w, h)
        if crop_paths is not None:
            cv2.imwrite(crop_paths[slot_index], crop)

        qr_text = _decode_text_from_crop(crop)
        if not qr_text:
            continue

        qr_list.append(
            {
                "text": qr_text,
                "center_x": x + w / 2.0,
                "center_y": y + h / 2.0,
                "slot": slot_index,
            }
        )

    return qr_list
