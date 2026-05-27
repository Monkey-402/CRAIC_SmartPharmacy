#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rospy

from move_nav.srv import Board1Decode, Board1DecodeResponse

from qr_decoder import decode_qr
from qr_parser import parse_qr


class QRNode:
    def __init__(self):
        rospy.init_node("qr_node")

        service_name = rospy.get_param(
            "~board1_decode_service",
            "/yaofang_vision/board1_decode",
        )
        self.service = rospy.Service(
            service_name,
            Board1Decode,
            self.handle_board1_decode,
        )

        rospy.loginfo("QR board1 decode service started: %s", service_name)

    def handle_board1_decode(self, req):
        rospy.loginfo("Receive board1 decode request: image_path=%s", req.image_path)

        image = cv2.imread(req.image_path)
        if image is None:
            rospy.logerr("Image load failed: %s", req.image_path)
            return Board1DecodeResponse(False, False, False, 0, 0)

        qr_list = decode_qr(image)
        rospy.loginfo("QR result: %s", qr_list)

        has_a, has_b, has_c, delivery_slot, sample_count = parse_qr(qr_list)

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
