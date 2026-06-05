#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time

import cv2
import rospy

from move_nav.srv import Board1Decode, Board1DecodeResponse

from qr_decoder import Board1DecodeParams, decode_qr
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
        self.decode_params = Board1DecodeParams.from_rosparam(rospy)
        self.service = rospy.Service(
            service_name,
            Board1Decode,
            self.handle_board1_decode,
        )

        rospy.loginfo("QR board1 decode service started: %s", service_name)
        rospy.loginfo(
            "Board1 decode: bright_thresh=%d min_bright_pixels=%d min_qr_area=%.0f",
            self.decode_params.bright_thresh,
            self.decode_params.min_bright_pixels,
            self.decode_params.min_qr_area,
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
                params=self.decode_params,
            )
        except Exception as exc:
            msg = "decode_exception: %s" % exc
            rospy.logerr("QR decode failed: %s", exc)
            return _failure_response(msg)

        if decode_result.error_message:
            rospy.logerr("[QR] %s", decode_result.error_message)
            meta = decode_result.meta or {}
            bc = meta.get("bright_center")
            if bc is not None:
                rospy.logerr(
                    "[QR] bright_center=(%.1f, %.1f) raw_hits=%d",
                    bc[0],
                    bc[1],
                    meta.get("raw_decode_count", 0),
                )
            return _failure_response(decode_result.error_message)

        qr_list = decode_result.qr_list
        meta = decode_result.meta or {}
        bc = meta.get("bright_center")
        if bc is not None:
            rospy.loginfo(
                "QR bright_center=(%.1f, %.1f) raw_hits=%d",
                bc[0],
                bc[1],
                meta.get("raw_decode_count", 0),
            )
        if req.image_path:
            base, _ext = os.path.splitext(req.image_path)
            rospy.loginfo("QR crops (if any): %s_slot[1-4].jpg", base)

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
            prefer_fewest = bool(getattr(req, "prefer_fewest_samples", False))
            prefer_sample_count = int(getattr(req, "prefer_sample_count", 0) or 0)
            has_a, has_b, has_c, delivery_slot, sample_count = parse_qr(
                qr_list,
                prefer_fewest=prefer_fewest,
                prefer_sample_count=prefer_sample_count,
            )
        except Exception as exc:
            msg = "parse_exception: %s" % exc
            rospy.logerr("QR parse failed: %s", exc)
            return _failure_response(msg)

        if sample_count == 0 or delivery_slot < 1 or delivery_slot > 4:
            msg = (
                "parse_failed: 已扫到 %d 个码但无有效任务（需含 A/B/C 且 slot 1-4），"
                "raw=%s" % (len(qr_list), qr_list)
            )
            rospy.logerr("[QR] %s", msg)
            return _failure_response(msg)

        rospy.loginfo(
            "Board1 decode OK (%d/%d slots, prefer_fewest=%s prefer_sample_count=%d): "
            "A=%s B=%s C=%s delivery_slot=%d sample_count=%d",
            len(qr_list),
            4,
            prefer_fewest,
            prefer_sample_count,
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
