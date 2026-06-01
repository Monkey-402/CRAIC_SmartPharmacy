#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""识别板一：检测四个黑框 → 分格裁剪 → 每格扫码。"""

import os

import cv2
from pyzbar.pyzbar import decode as pyzbar_decode


class FrameDetectParams(object):
    """黑框检测阈值（默认略放宽；可在 launch 中覆盖）。"""

    def __init__(
        self,
        min_area_ratio=0.008,
        max_area_ratio=0.48,
        inner_max_area_ratio=0.42,
        aspect_min=0.50,
        aspect_max=1.85,
        min_side=20,
        approx_eps_ratio=0.05,
        area_similar_min=0.35,
        area_similar_max=4.0,
        dedupe_dist_thresh=20,
        crop_margin_ratio=0.01,
        morph_close_iters=2,
    ):
        self.min_area_ratio = min_area_ratio
        self.max_area_ratio = max_area_ratio
        self.inner_max_area_ratio = inner_max_area_ratio
        self.aspect_min = aspect_min
        self.aspect_max = aspect_max
        self.min_side = min_side
        self.approx_eps_ratio = approx_eps_ratio
        self.area_similar_min = area_similar_min
        self.area_similar_max = area_similar_max
        self.dedupe_dist_thresh = dedupe_dist_thresh
        self.crop_margin_ratio = crop_margin_ratio
        self.morph_close_iters = morph_close_iters

    @classmethod
    def from_rosparam(cls, node_handle=None):
        if node_handle is None:
            import rospy
            node_handle = rospy

        return cls(
            min_area_ratio=node_handle.get_param("~min_area_ratio", 0.008),
            max_area_ratio=node_handle.get_param("~max_area_ratio", 0.48),
            inner_max_area_ratio=node_handle.get_param(
                "~inner_max_area_ratio", 0.42
            ),
            aspect_min=node_handle.get_param("~aspect_min", 0.50),
            aspect_max=node_handle.get_param("~aspect_max", 1.85),
            min_side=int(node_handle.get_param("~min_side", 20)),
            approx_eps_ratio=node_handle.get_param("~approx_eps_ratio", 0.05),
            area_similar_min=node_handle.get_param("~area_similar_min", 0.35),
            area_similar_max=node_handle.get_param("~area_similar_max", 4.0),
            dedupe_dist_thresh=node_handle.get_param("~dedupe_dist_thresh", 20),
            crop_margin_ratio=node_handle.get_param("~crop_margin_ratio", 0.01),
            morph_close_iters=int(
                node_handle.get_param("~morph_close_iters", 2)
            ),
        )


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


def _crop_inset(image, x, y, w, h, margin_ratio):
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


def _is_squareish(w, h, params):
    if h <= 0:
        return False
    ratio = w / float(h)
    return params.aspect_min <= ratio <= params.aspect_max


def _rect_from_contour(cnt, params):
    peri = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, params.approx_eps_ratio * peri, True)
    if len(approx) == 4 and cv2.isContourConvex(approx):
        x, y, w, h = cv2.boundingRect(approx)
    else:
        x, y, w, h = cv2.boundingRect(cnt)

    if w < params.min_side or h < params.min_side:
        return None
    if not _is_squareish(w, h, params):
        return None
    return (x, y, w, h)


def _collect_frame_rects(contours, img_area, params):
    rects = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < img_area * params.min_area_ratio:
            continue
        if area > img_area * params.max_area_ratio:
            continue
        rect = _rect_from_contour(cnt, params)
        if rect is not None:
            rects.append(rect)
    return rects


def _dedupe_rects(rects, dist_thresh):
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


def _group_four_similar_by_area(rects, params):
    """Pick 4 rects with mutually similar area (smallest qualifying group first)."""
    if len(rects) < 4:
        return None

    by_area = sorted(rects, key=lambda r: r[2] * r[3])
    for start in range(len(by_area) - 3):
        group = by_area[start : start + 4]
        areas = [r[2] * r[3] for r in group]
        ref = max(areas)
        if min(areas) < ref * params.area_similar_min:
            continue
        if max(areas) > ref * params.area_similar_max:
            continue
        return _sort_frames_to_slots(group)

    smallest = by_area[:4]
    areas = [r[2] * r[3] for r in smallest]
    ref = max(areas)
    if min(areas) >= ref * params.area_similar_min:
        return _sort_frames_to_slots(smallest)
    return None


def _pick_four_frame_rects(rects, img_area, params):
    rects = _dedupe_rects(rects, params.dedupe_dist_thresh)
    if len(rects) < 4:
        return None

    inner = [
        r for r in rects
        if r[2] * r[3] < img_area * params.inner_max_area_ratio
    ]
    if len(inner) >= 4:
        rects = inner

    if len(rects) == 4:
        return _sort_frames_to_slots(rects)

    # Poster / floor blobs are often largest; four cells are a similar smaller cluster.
    grouped = _group_four_similar_by_area(rects, params)
    if grouped is not None:
        return grouped

    rects = sorted(rects, key=lambda r: r[2] * r[3], reverse=True)
    ref_area = rects[0][2] * rects[0][3]
    similar = [
        r for r in rects
        if ref_area * params.area_similar_min
        <= (r[2] * r[3])
        <= ref_area * params.area_similar_max
    ]
    if len(similar) < 4:
        return None

    similar = sorted(similar, key=lambda r: r[2] * r[3], reverse=True)[:4]
    return _sort_frames_to_slots(similar)


def detect_four_frames(image, params=None):
    if params is None:
        params = FrameDetectParams()
    if image is None or image.size == 0:
        return None

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, inv = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    img_area = float(gray.shape[0] * gray.shape[1])
    kernel_sizes = (5, 9, 13, 15)

    for ksize in kernel_sizes:
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (ksize, ksize))
        mask = cv2.morphologyEx(
            inv,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=params.morph_close_iters,
        )
        contours = _find_contours(mask)
        rects = _collect_frame_rects(contours, img_area, params)
        picked = _pick_four_frame_rects(rects, img_area, params)
        if picked is not None:
            return picked

    contours = _find_contours(inv)
    rects = _collect_frame_rects(contours, img_area, params)
    return _pick_four_frame_rects(rects, img_area, params)


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


class QrDecodeResult(object):
    """二维码解码结果；error_message 为空表示 parse 成功。"""

    __slots__ = ("qr_list", "error_message", "frames_detected")

    def __init__(self, qr_list=None, error_message="", frames_detected=False):
        self.qr_list = qr_list or []
        self.error_message = error_message
        self.frames_detected = frames_detected


def decode_qr(image, source_image_path=None, params=None):
    if params is None:
        params = FrameDetectParams()

    crop_paths = None
    if source_image_path:
        crop_paths = _crop_paths_for_source(source_image_path)
        _ensure_dir(os.path.dirname(source_image_path))

    frames = detect_four_frames(image, params)
    if frames is None:
        return QrDecodeResult(
            error_message="frame_detect_failed: 未检测到四个黑框区域",
            frames_detected=False,
        )

    qr_list = []
    decoded_any = False
    for slot_index, (x, y, w, h) in enumerate(frames, start=1):
        crop = _crop_inset(image, x, y, w, h, params.crop_margin_ratio)
        if crop_paths is not None:
            cv2.imwrite(crop_paths[slot_index], crop)

        qr_text = _decode_text_from_crop(crop)
        if not qr_text:
            continue

        decoded_any = True
        qr_list.append(
            {
                "text": qr_text,
                "center_x": x + w / 2.0,
                "center_y": y + h / 2.0,
                "slot": slot_index,
            }
        )

    if not decoded_any:
        return QrDecodeResult(
            qr_list=[],
            error_message="decode_failed: 已检出四格黑框，但所有格子 pyzbar 均未扫到码",
            frames_detected=True,
        )

    return QrDecodeResult(qr_list=qr_list, frames_detected=True)
