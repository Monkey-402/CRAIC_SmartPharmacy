#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time

import cv2
import rospy

from move_nav.srv import Board1Decode, Board1DecodeResponse

from qr_decoder import FrameDetectParams, decode_qr
from qr_parser import parse_qr


def _monotonic():
    try:
        return time.monotonic()
    except AttributeError:
        return time.time()


def _read_image_when_ready(image_path, timeout_sec, poll_sec):
    deadline = _monotonic() + timeout_sec
    last_size = -1

    while _monotonic() < deadline:
        try:
            current_size = os.path.getsize(image_path)
        except OSError:
            current_size = 0

        if current_size > 0 and current_size == last_size:
            image = cv2.imread(image_path)
            if image is not None:
                return image

        last_size = current_size
        time.sleep(poll_sec)

    return cv2.imread(image_path)


def _failure_response(error_message):
    return Board1DecodeResponse(
        False, False, False, 0, 0, error_message,
    )


class QRNode:
    def __init__(self):
        rospy.init_node("qr_node")

        service_name = rospy.get_param(
            "~board1_decode_service",
            "/yaofang_vision/board1_decode",
        )
        self.image_ready_timeout_sec = rospy.get_param(
            "~image_ready_timeout_sec",
            0.2,
        )
        self.image_ready_poll_sec = rospy.get_param(
            "~image_ready_poll_sec",
            0.01,
        )
        self.frame_params = FrameDetectParams.from_rosparam(rospy)
        self.service = rospy.Service(
            service_name,
            Board1Decode,
            self.handle_board1_decode,
        )

        rospy.loginfo("QR board1 decode service started: %s", service_name)
        rospy.loginfo(
            "Frame detect params: aspect=[%.2f, %.2f] area=[%.3f, %.3f] "
            "crop_margin=%.2f",
            self.frame_params.aspect_min,
            self.frame_params.aspect_max,
            self.frame_params.min_area_ratio,
            self.frame_params.max_area_ratio,
            self.frame_params.crop_margin_ratio,
        )

    def handle_board1_decode(self, req):
        rospy.loginfo("Receive board1 decode request: image_path=%s", req.image_path)

        image = _read_image_when_ready(
            req.image_path,
            self.image_ready_timeout_sec,
            self.image_ready_poll_sec,
        )
        if image is None:
            msg = "image_load_failed: 无法读取图片 %s" % req.image_path
            rospy.logerr(msg)
            return _failure_response(msg)

        try:
            decode_result = decode_qr(
                image,
                source_image_path=req.image_path,
                params=self.frame_params,
            )
        except Exception as exc:
            msg = "decode_exception: %s" % exc
            rospy.logerr("QR decode failed: %s", exc)
            return _failure_response(msg)

        if decode_result.error_message:
            if decode_result.error_message.startswith("frame_detect_failed"):
                rospy.logerr("[QR] %s", decode_result.error_message)
            elif decode_result.error_message.startswith("decode_failed"):
                rospy.logerr("[QR] %s", decode_result.error_message)
                if req.image_path:
                    base, _ext = os.path.splitext(req.image_path)
                    rospy.logerr(
                        "[QR] 已保存裁剪图供排查: %s_slot[1-4].jpg", base
                    )
            return _failure_response(decode_result.error_message)

        qr_list = decode_result.qr_list
        if req.image_path:
            base, _ext = os.path.splitext(req.image_path)
            rospy.loginfo("Frame crops saved: %s_slot[1-4].jpg", base)

        rospy.loginfo("QR raw result: %s", qr_list)
        for qr in qr_list:
            rospy.loginfo(
                "QR detected: text=%s center=(%.1f, %.1f) slot=%d",
                qr["text"],
                qr["center_x"],
                qr["center_y"],
                qr["slot"],
            )

        try:
            has_a, has_b, has_c, delivery_slot, sample_count = parse_qr(qr_list)
        except Exception as exc:
            msg = "parse_exception: %s" % exc
            rospy.logerr("QR parse failed: %s", exc)
            return _failure_response(msg)

        if sample_count == 0 or delivery_slot < 1 or delivery_slot > 4:
            msg = (
                "parse_failed: 已扫到码但内容无效（需含 A/B/C 且 slot 1-4），"
                "raw=%s" % qr_list
            )
            rospy.logerr("[QR] %s", msg)
            return _failure_response(msg)

        rospy.loginfo(
            "Board1 decode result: A=%s B=%s C=%s delivery_slot=%d sample_count=%d",
            has_a,
            has_b,
            has_c,
            delivery_slot,
            sample_count,
        )
        return Board1DecodeResponse(
            has_a,
            has_b,
            has_c,
            delivery_slot,
            sample_count,
            "",
        )


if __name__ == "__main__":
    QRNode()
    rospy.spin()
