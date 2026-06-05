#!/usr/bin/env python
# -*- coding: utf-8 -*-


def _qr_text(qr):
    if isinstance(qr, dict):
        return qr.get("text", "")
    return str(qr)


def _qr_slot(qr):
    if isinstance(qr, dict):
        try:
            return int(qr.get("slot", 0))
        except (TypeError, ValueError):
            return 0
    return 0


def _sample_flags(text):
    # 二维码内容按比赛规则只包含 A/B/C；重复字母只按一个样本窗口计数。
    text_upper = text.upper()
    has_a = "A" in text_upper
    has_b = "B" in text_upper
    has_c = "C" in text_upper
    sample_count = int(has_a) + int(has_b) + int(has_c)
    return has_a, has_b, has_c, sample_count


def parse_qr(qr_list, prefer_fewest=False, prefer_sample_count=0):
    candidates = []

    for qr in qr_list:
        delivery_slot = _qr_slot(qr)
        has_a, has_b, has_c, sample_count = _sample_flags(_qr_text(qr))

        # 过滤掉无法定位到 1-4 位置、或内容里没有 A/B/C 的识别结果。
        if delivery_slot < 1 or delivery_slot > 4 or sample_count == 0:
            continue

        candidates.append(
            {
                "has_a": has_a,
                "has_b": has_b,
                "has_c": has_c,
                "delivery_slot": delivery_slot,
                "sample_count": sample_count,
            }
        )

    if not candidates:
        return False, False, False, 0, 0

    target = int(prefer_sample_count or 0)
    if target > 0:
        # 双车最后一轮等：优先选恰好 N 个样本的码；无则选与 N 最接近的（同距取样本较多）。
        exact = [c for c in candidates if c["sample_count"] == target]
        if exact:
            selected = min(exact, key=lambda c: c["delivery_slot"])
        else:
            selected = min(
                candidates,
                key=lambda c: (
                    abs(c["sample_count"] - target),
                    -c["sample_count"],
                    c["delivery_slot"],
                ),
            )
    elif prefer_fewest:
        selected = min(
            candidates,
            key=lambda candidate: (
                candidate["sample_count"],
                candidate["delivery_slot"],
            ),
        )
    else:
        selected = max(
            candidates,
            key=lambda candidate: (
                candidate["sample_count"],
                -candidate["delivery_slot"],
            ),
        )
    return (
        selected["has_a"],
        selected["has_b"],
        selected["has_c"],
        selected["delivery_slot"],
        selected["sample_count"],
    )
