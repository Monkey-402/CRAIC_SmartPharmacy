#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pyzbar.pyzbar import decode


def _center_from_result(result):
    # pyzbar 通常会给出二维码四个角点；用角点平均值比 rect 中心更贴近旋转后的二维码中心。
    if result.polygon:
        x_sum = sum(point.x for point in result.polygon)
        y_sum = sum(point.y for point in result.polygon)
        point_count = len(result.polygon)
        return float(x_sum) / point_count, float(y_sum) / point_count

    # 没有 polygon 时退回到外接矩形中心。
    rect = result.rect
    return rect.left + rect.width / 2.0, rect.top + rect.height / 2.0


def _slot_from_center(center_x, center_y, image_width, image_height):
    # 比赛识别板按整张图的 2x2 象限编号：左上 1、右上 2、左下 3、右下 4。
    in_left_half = center_x < image_width / 2.0
    in_top_half = center_y < image_height / 2.0

    if in_left_half and in_top_half:
        return 1
    if not in_left_half and in_top_half:
        return 2
    if in_left_half and not in_top_half:
        return 3
    return 4


def decode_qr(image):
    # 返回结构化结果，后续解析时既需要二维码文本，也需要中心点推断出的 slot。
    try:
        results = decode(image)
    except Exception:
        return []

    qr_list = []
    image_height, image_width = image.shape[:2]

    for r in results:
        try:
            qr_text = r.data.decode("utf-8")
            center_x, center_y = _center_from_result(r)
        except (AttributeError, UnicodeDecodeError, ZeroDivisionError):
            # 单个二维码内容或定位信息异常时跳过，不影响同一张图里的其他二维码。
            continue

        qr_list.append(
            {
                "text": qr_text,
                "center_x": center_x,
                "center_y": center_y,
                "slot": _slot_from_center(
                    center_x,
                    center_y,
                    image_width,
                    image_height,
                ),
            }
        )

    return qr_list
