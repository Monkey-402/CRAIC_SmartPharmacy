#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import re
import sys

import cv2
import numpy as np
import pytesseract
import rospy

from move_nav.srv import Board2Decode, Board2DecodeResponse

try:
    text_type = unicode
    binary_type = str
except NameError:
    text_type = str
    binary_type = bytes


class OCRService(object):
    def __init__(self):
        rospy.init_node("ocr_service", anonymous=True)

        service_name = rospy.get_param(
            "~board2_decode_service",
            "/yaofang_vision/board2_decode",
        )

        self._warm_up_tesseract()
        self.service = rospy.Service(
            service_name,
            Board2Decode,
            self.handle_request,
        )
        rospy.loginfo("OCR board2 decode service started: %s", service_name)

    def _warm_up_tesseract(self):
        rospy.loginfo("Loading Tesseract OCR engine...")
        dummy_img = np.zeros((100, 100), dtype=np.uint8)
        pytesseract.image_to_string(dummy_img, lang="chi_sim+eng")
        rospy.loginfo("Tesseract OCR engine ready")

    def _to_text(self, value):
        if isinstance(value, text_type):
            return value
        if isinstance(value, binary_type):
            return value.decode("utf-8", "ignore")
        return text_type(value)

    def _to_ros_string(self, value):
        if sys.version_info[0] < 3 and isinstance(value, text_type):
            return value.encode("utf-8")
        return value

    def extract_wait_seconds(self, text):
        if not text:
            return 0

        normalized = re.sub(r"\s+", " ", text)
        numbers = re.findall(r"(\d+)", normalized)
        if not numbers:
            return 0

        seconds = int(numbers[-1])
        rospy.loginfo("Extracted wait seconds: %d", seconds)
        return seconds

    def handle_request(self, req):
        wait_seconds = 0
        speech_text = ""

        if not req.image_path or not os.path.exists(req.image_path):
            rospy.logwarn("Image does not exist: %s", req.image_path)
            return Board2DecodeResponse(wait_seconds, speech_text)

        try:
            cv_image = cv2.imread(req.image_path)
            if cv_image is None:
                rospy.logerr("Failed to read image: %s", req.image_path)
                return Board2DecodeResponse(wait_seconds, speech_text)

            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            text_raw = pytesseract.image_to_string(gray, lang="chi_sim+eng")
            speech_text = self._to_text(text_raw).strip()
            wait_seconds = self.extract_wait_seconds(speech_text)

            rospy.loginfo(
                "Board2 OCR result: wait_seconds=%d speech_text=%s",
                wait_seconds,
                speech_text.encode("utf-8") if isinstance(speech_text, text_type) else speech_text,
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
