#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""识别板二：检测单个 A4 比例（√2:1）黑框矩形 → 向内裁剪屏幕/告示区域。"""

import math

import cv2

# A4 长边/短边 ≈ 1.414（横版 w/h 或竖版 h/w）
A4_ASPECT_RATIO = math.sqrt(2.0)


class A4FrameParams(object):
    def __init__(
        self,
        min_area_ratio=0.04,
        max_area_ratio=0.75,
        a4_aspect_ratio=A4_ASPECT_RATIO,
        aspect_tol=0.22,
        min_side=50,
        approx_eps_ratio=0.04,
        crop_margin_ratio=0.04,
        morph_close_iters=2,
    ):
        self.min_area_ratio = min_area_ratio
        self.max_area_ratio = max_area_ratio
        self.a4_aspect_ratio = a4_aspect_ratio
        self.aspect_tol = aspect_tol
        self.min_side = min_side
        self.approx_eps_ratio = approx_eps_ratio
        self.crop_margin_ratio = crop_margin_ratio
        self.morph_close_iters = morph_close_iters

    @classmethod
    def from_rosparam(cls, node_handle=None):
        if node_handle is None:
            import rospy
            node_handle = rospy

        return cls(
            min_area_ratio=node_handle.get_param("~frame_min_area_ratio", 0.04),
            max_area_ratio=node_handle.get_param("~frame_max_area_ratio", 0.75),
            a4_aspect_ratio=float(
                node_handle.get_param("~frame_a4_aspect_ratio", A4_ASPECT_RATIO)
            ),
            aspect_tol=node_handle.get_param("~frame_aspect_tol", 0.22),
            min_side=int(node_handle.get_param("~frame_min_side", 50)),
            approx_eps_ratio=node_handle.get_param("~frame_approx_eps_ratio", 0.04),
            crop_margin_ratio=node_handle.get_param("~frame_crop_margin_ratio", 0.04),
            morph_close_iters=int(
                node_handle.get_param("~frame_morph_close_iters", 2)
            ),
        )


class A4FrameResult(object):
    __slots__ = ("rect", "error_message")

    def __init__(self, rect=None, error_message=""):
        self.rect = rect
        self.error_message = error_message

    @property
    def ok(self):
        return self.rect is not None and not self.error_message


def _find_contours(binary):
    result = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(result) == 3:
        _image, contours, _hierarchy = result
    else:
        contours, _hierarchy = result
    return contours


def _a4_aspect_ok(w, h, params):
    if w <= 0 or h <= 0:
        return False
    long_side = max(w, h)
    short_side = min(w, h)
    ratio = long_side / float(short_side)
    target = params.a4_aspect_ratio
    return abs(ratio - target) <= params.aspect_tol * target


def _rect_from_contour(cnt, params):
    peri = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, params.approx_eps_ratio * peri, True)
    if len(approx) == 4 and cv2.isContourConvex(approx):
        x, y, w, h = cv2.boundingRect(approx)
    else:
        x, y, w, h = cv2.boundingRect(cnt)

    if w < params.min_side or h < params.min_side:
        return None
    if not _a4_aspect_ok(w, h, params):
        return None
    return (x, y, w, h)


def _aspect_error(w, h, params):
    ratio = max(w, h) / float(min(w, h))
    return abs(ratio - params.a4_aspect_ratio)


def _collect_a4_rects(contours, img_area, params):
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


def _pick_best_a4_rect(rects, params):
    if not rects:
        return None
    return min(
        rects,
        key=lambda r: (_aspect_error(r[2], r[3], params), -(r[2] * r[3])),
    )


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


def detect_a4_frame(image, params=None):
    if params is None:
        params = A4FrameParams()
    if image is None or image.size == 0:
        return A4FrameResult(error_message="frame_detect_failed: 空图像")

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, inv = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    img_area = float(gray.shape[0] * gray.shape[1])
    best_rect = None

    for ksize in (5, 9, 13, 15, 19):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (ksize, ksize))
        mask = cv2.morphologyEx(
            inv,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=params.morph_close_iters,
        )
        contours = _find_contours(mask)
        rects = _collect_a4_rects(contours, img_area, params)
        candidate = _pick_best_a4_rect(rects, params)
        if candidate is not None:
            best_rect = candidate
            break

    if best_rect is None:
        contours = _find_contours(inv)
        rects = _collect_a4_rects(contours, img_area, params)
        best_rect = _pick_best_a4_rect(rects, params)

    if best_rect is None:
        return A4FrameResult(
            error_message="frame_detect_failed: 未检测到 A4 比例黑框",
        )

    return A4FrameResult(rect=best_rect)


def crop_a4_frame(image, params=None):
    result = detect_a4_frame(image, params)
    if not result.ok:
        return result, None
    x, y, w, h = result.rect
    crop = _crop_inset(image, x, y, w, h, params.crop_margin_ratio)
    return result, crop


def draw_frame_box(image, rect, color=(0, 255, 0), thickness=2):
    if image is None or rect is None:
        return image
    vis = image.copy()
    x, y, w, h = rect
    cv2.rectangle(vis, (x, y), (x + w, y + h), color, thickness)
    return vis
