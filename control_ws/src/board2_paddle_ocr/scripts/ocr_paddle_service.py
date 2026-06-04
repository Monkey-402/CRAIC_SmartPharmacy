#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

import rospy

from move_nav.srv import Board2Decode, Board2DecodeResponse

from paddle_ocr_client import board2_decode_http, health_check

try:
    text_type = unicode
    binary_type = str
except NameError:
    text_type = str
    binary_type = bytes


class PaddleOCRService(object):
    def __init__(self):
        rospy.init_node("ocr_paddle_service", anonymous=True)

        service_name = rospy.get_param(
            "~board2_decode_service",
            "/yaofang_vision/board2_decode",
        )
        self.paddle_url = rospy.get_param(
            "~paddle_ocr_url",
            os.environ.get("PADDLE_OCR_URL", "http://127.0.0.1:8765"),
        )
        self.timeout = float(rospy.get_param("~paddle_ocr_timeout", 120.0))
        self.use_keyword_inference = bool(
            rospy.get_param("~use_keyword_inference", True)
        )
        self.check_health_on_start = bool(
            rospy.get_param("~check_health_on_start", True)
        )

        if self.check_health_on_start:
            if health_check(self.paddle_url, timeout=5.0):
                rospy.loginfo("Paddle OCR HTTP service is up: %s", self.paddle_url)
            else:
                rospy.logwarn(
                    "Paddle OCR HTTP service not reachable at %s — "
                    "请先在本机运行: roscd board2_paddle_ocr && ./run_paddle_ocr_server.sh",
                    self.paddle_url,
                )

        self.service = rospy.Service(
            service_name,
            Board2Decode,
            self.handle_request,
        )
        rospy.loginfo(
            "Paddle board2 decode service started: %s (url=%s timeout=%.1fs)",
            service_name,
            self.paddle_url,
            self.timeout,
        )

    def _to_text(self, value):
        if value is None:
            return ""
        if isinstance(value, text_type):
            return value
        if isinstance(value, binary_type):
            return value.decode("utf-8", "ignore")
        return text_type(value)

    def _to_ros_string(self, value):
        value = self._to_text(value)
        if sys.version_info[0] < 3 and isinstance(value, text_type):
            return value.encode("utf-8")
        return value

    def _to_log_string(self, value):
        value = self._to_text(value)
        if sys.version_info[0] < 3 and isinstance(value, text_type):
            return value.encode("utf-8")
        return value

    def handle_request(self, req):
        wait_seconds = 0
        speech_text = ""

        if not req.image_path or not os.path.exists(req.image_path):
            rospy.logwarn("Image does not exist: %s", req.image_path)
            return Board2DecodeResponse(wait_seconds, speech_text)

        result = board2_decode_http(
            req.image_path,
            base_url=self.paddle_url,
            timeout=self.timeout,
            use_keyword_inference=self.use_keyword_inference,
        )

        if not result["ok"]:
            rospy.logerr(
                "Paddle OCR failed: %s",
                self._to_log_string(result.get("error", "unknown")),
            )
            return Board2DecodeResponse(wait_seconds, speech_text)

        speech_text = self._to_ros_string(result["speech_text"])
        wait_seconds = int(result.get("wait_seconds", 0) or 0)

        rospy.loginfo(
            "Board2 Paddle OCR result: wait_seconds=%d speech_text=%s (raw=%s)",
            wait_seconds,
            self._to_log_string(result["speech_text"]),
            self._to_log_string(result["raw_text"]),
        )
        return Board2DecodeResponse(wait_seconds, speech_text)


if __name__ == "__main__":
    try:
        PaddleOCRService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
