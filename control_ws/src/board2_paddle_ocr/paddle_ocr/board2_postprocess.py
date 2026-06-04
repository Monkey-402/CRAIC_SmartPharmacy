# -*- coding: utf-8 -*-
"""板二 OCR 后处理：与 text_recognition/ocr_service 关键词推断一致（无 ROS 依赖）。"""

from __future__ import print_function

import re
import sys
import unicodedata

try:
    text_type = unicode
except NameError:
    text_type = str


def _to_text(value):
    if value is None:
        return ""
    if isinstance(value, text_type):
        return value
    if isinstance(value, bytes):
        return value.decode("utf-8", "ignore")
    return text_type(value)


def extract_wait_seconds(text):
    if not text:
        return 0

    normalized = unicodedata.normalize("NFKC", _to_text(text))
    normalized = " ".join(normalized.split())

    if sys.version_info[0] < 3:
        regex_text = normalized.encode("utf-8")
        regex_pattern = "(\\d+)\\s*(?:\xe7\xa7\x92|s\\b|sec(?:ond)?s?\\b)"
    else:
        regex_text = normalized
        regex_pattern = r"(\d+)\s*(?:秒|s\b|sec(?:ond)?s?\b)"

    matches = re.findall(regex_pattern, regex_text, re.IGNORECASE)
    if not matches:
        return 0
    return int(matches[0])


def infer_board2_speech(raw_text, use_keyword_inference=True):
    if not use_keyword_inference:
        return _to_text(raw_text).strip()

    normalized = unicodedata.normalize("NFKC", _to_text(raw_text))
    normalized = normalized.replace(" ", "")

    has_lab = (
        (u"化验" in normalized)
        or (u"化" in normalized and u"验" in normalized)
        or (u"化" in normalized)
    )
    has_idle = (
        (u"空闲" in normalized)
        or (u"空" in normalized and u"闲" in normalized)
        or (u"快速" in normalized)
    )
    has_busy = (
        (u"忙碌" in normalized)
        or (u"忙" in normalized)
        or (u"等待" in normalized)
    )

    if has_lab and has_idle:
        return u"化验区空闲中，请快速通过"
    if has_lab and has_busy:
        seconds = extract_wait_seconds(normalized)
        if seconds > 0:
            return u"化验区忙碌中，需等待 %d 秒" % seconds
        return u"化验区忙碌中，需等待"
    if has_idle:
        return u"化验区空闲中，请快速通过"
    if has_busy:
        seconds = extract_wait_seconds(normalized)
        if seconds > 0:
            return u"化验区忙碌中，需等待 %d 秒" % seconds

    return _to_text(raw_text).strip()


def board2_result_from_raw(raw_text, use_keyword_inference=True):
    speech_text = infer_board2_speech(raw_text, use_keyword_inference)
    wait_seconds = extract_wait_seconds(speech_text)
    if wait_seconds == 0 and speech_text != _to_text(raw_text).strip():
        wait_seconds = extract_wait_seconds(raw_text)
    return {
        "raw_text": _to_text(raw_text).strip(),
        "speech_text": _to_text(speech_text).strip(),
        "wait_seconds": wait_seconds,
    }
