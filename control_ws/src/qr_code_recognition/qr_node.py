#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time

import cv2
import rospy

from move_nav.srv import Board1Decode, Board1DecodeResponse

from qr_decoder import decode_qr
from qr_parser import parse_qr


def _read_image_when_ready(image_path, timeout_sec, poll_sec):
    deadline = time.monotonic() + timeout_sec
    last_size = -1

    while time.monotonic() < deadline:
        try:
            current_size = os.path.getsize(image_path)
        except OSError:
            current_size = 0

        # 等到文件非空且连续两次轮询大小一致，再认为 cv2.imwrite 基本完成。
        if current_size > 0 and current_size == last_size:
            image = cv2.imread(image_path)
            if image is not None:
                return image

        last_size = current_size
        time.sleep(poll_sec)

    return cv2.imread(image_path)


# ROS service 入口：调用方传入图片路径，节点返回一个 Board1Decode 结构化结果。
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
        self.service = rospy.Service(
            service_name,
            Board1Decode,
            self.handle_board1_decode,
        )

        rospy.loginfo("QR board1 decode service started: %s", service_name)

    def handle_board1_decode(self, req):
        rospy.loginfo("Receive board1 decode request: image_path=%s", req.image_path)

        image = _read_image_when_ready(
            req.image_path,
            self.image_ready_timeout_sec,
            self.image_ready_poll_sec,
        )
        if image is None:
            rospy.logerr("Image load failed: %s", req.image_path)
            return Board1DecodeResponse(False, False, False, 0, 0)

        # decode_qr 负责识别所有二维码并根据中心点推断 slot；parse_qr 再按比赛规则选出一个结果。
        try:
            qr_list = decode_qr(image)
        except Exception as exc:
            rospy.logerr("QR decode failed: %s", exc)
            return Board1DecodeResponse(False, False, False, 0, 0)

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
            rospy.logerr("QR parse failed: %s", exc)
            return Board1DecodeResponse(False, False, False, 0, 0)

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
        )


if __name__ == "__main__":
    QRNode()
    rospy.spin()
