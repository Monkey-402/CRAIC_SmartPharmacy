#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
from pyzbar.pyzbar import decode


def decode_qr(image):

    results = decode(image)

    qr_list = []

    for r in results:
        qr_data = r.data.decode("utf-8")
        qr_list.append(qr_data)

    return qr_list
