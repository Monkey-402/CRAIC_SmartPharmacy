#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import sys
import unicodedata

import cv2
import numpy as np
import pytesseract
import rospy

from move_nav.srv import Board2Decode, Board2DecodeResponse

# 兼容 ROS Melodic 常见的 Python 2 环境，以及较新的 Python 3 环境。
try:
    text_type = unicode
    binary_type = str
except NameError:
    text_type = str
    binary_type = bytes


class OCRService(object):
    def __init__(self):
        rospy.init_node("ocr_service", anonymous=True)

        # 服务名可以在 launch 中重映射；默认值用于药房 board2 流程。
        service_name = rospy.get_param(
            "~board2_decode_service",
            "/yaofang_vision/board2_decode",
        )
        # 默认使用大津法；如需改为自适应二值化，在 launch 中设置 threshold_method:=adaptive。
        self.threshold_method = rospy.get_param("~threshold_method", "otsu")
        self.adaptive_block_size = int(rospy.get_param("~adaptive_block_size", 31))
        self.adaptive_c = int(rospy.get_param("~adaptive_c", 11))

        self._warm_up_tesseract()
        self.service = rospy.Service(
            service_name,
            Board2Decode,
            self.handle_request,
        )
        rospy.loginfo("OCR board2 decode service started: %s", service_name)

    def _warm_up_tesseract(self):
        # 启动时先调用一次 Tesseract，避免第一次真实请求承担模型加载耗时。
        rospy.loginfo("Loading Tesseract OCR engine...")
        dummy_img = np.zeros((100, 100), dtype=np.uint8)
        pytesseract.image_to_string(dummy_img, lang="chi_sim+eng")
        rospy.loginfo("Tesseract OCR engine ready")

    # 将输入的二进制数据或普通字符串安全地转换成统一的 Unicode 文本
    def _to_text(self, value):
        if value is None:
            return ""
        if isinstance(value, text_type):
            return value
        if isinstance(value, binary_type):
            return value.decode("utf-8", "ignore")
        return text_type(value)

    # 防止中文乱码以及防止 Python 2 下的类型报错
    def _to_ros_string(self, value):
        value = self._to_text(value)
        if sys.version_info[0] < 3 and isinstance(value, text_type):
            return value.encode("utf-8")
        return value

    # 日志输出使用安全字符串，避免空识别结果或编码问题影响服务返回。
    def _to_log_string(self, value):
        value = self._to_text(value)
        if sys.version_info[0] < 3 and isinstance(value, text_type):
            return value.encode("utf-8")
        return value

    # 针对屏幕反光、光照不均的场景做 OCR 前处理
    def _preprocess_for_ocr(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)

        if self.threshold_method == "otsu":
            # 大津法适合整体亮度比较均匀的图片。
            _, binary = cv2.threshold(
                blurred,
                0,
                255,
                cv2.THRESH_BINARY + cv2.THRESH_OTSU,
            )
        else:
            block_size = self._normalize_block_size(self.adaptive_block_size)
            # 自适应二值化更适合药机屏幕反光、局部阴影等光照不均的情况。
            binary = cv2.adaptiveThreshold(
                blurred,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                block_size,
                self.adaptive_c,
            )

        return binary

    # OpenCV 要求自适应二值化的 block size 必须是大于 1 的奇数。
    def _normalize_block_size(self, block_size):
        if block_size < 3:
            block_size = 3
        if block_size % 2 == 0:
            block_size += 1
        return block_size

    # 提取等待秒数
    def extract_wait_seconds(self, text):
        if not text:
            return 0

        # 只提取带有时间单位的数字，避免把窗口号、取药口、编号等误判为等待时间。
        normalized = unicodedata.normalize("NFKC", self._to_text(text))
        normalized = " ".join(normalized.split())

        if sys.version_info[0] < 3:
            # Python 2 下文本和中文单位都使用 UTF-8 字节流，避免 Unicode 正则匹配不稳定。
            regex_text = normalized.encode("utf-8")
            regex_pattern = "(\\d+)\\s*(?:\xe7\xa7\x92|s\\b|sec(?:ond)?s?\\b)"
        else:
            regex_text = normalized
            regex_pattern = u"(\\d+)\\s*(?:秒|s\\b|sec(?:ond)?s?\\b)"

        matches = re.findall(regex_pattern, regex_text, re.IGNORECASE)
        if not matches:
            return 0

        seconds = int(matches[0])
        rospy.loginfo("Extracted wait seconds: %d", seconds)
        return seconds

    # 服务回调函数
    def handle_request(self, req):
        wait_seconds = 0
        speech_text = ""

        # 服务接口传入的是图片路径，不是 ROS Image 消息；先校验文件再调用 OpenCV/Tesseract。
        if not req.image_path or not os.path.exists(req.image_path):
            rospy.logwarn("Image does not exist: %s", req.image_path)
            return Board2DecodeResponse(wait_seconds, speech_text)

        try:
            cv_image = cv2.imread(req.image_path)
            if cv_image is None:
                rospy.logerr("Failed to read image: %s", req.image_path)
                return Board2DecodeResponse(wait_seconds, speech_text)

            processed = self._preprocess_for_ocr(cv_image)
            text_raw = pytesseract.image_to_string(processed, lang="chi_sim+eng")
            speech_text = self._to_text(text_raw).strip()
            wait_seconds = self.extract_wait_seconds(speech_text)

            rospy.loginfo(
                "Board2 OCR result: wait_seconds=%d speech_text=%s",
                wait_seconds,
                self._to_log_string(speech_text),
            )
        except Exception as exc:
            rospy.logerr("OCR failed: %s", exc)

        return Board2DecodeResponse(wait_seconds, self._to_ros_string(speech_text))


if __name__ == "__main__":
    try:
        OCRService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
