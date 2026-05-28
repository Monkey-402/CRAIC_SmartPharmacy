#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""循环发布静态图片，模拟摄像头（默认话题 /yaofang_test/image_raw）。"""

import os

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class TestImagePublisher(object):
    def __init__(self):
        rospy.init_node("test_image_publisher", anonymous=False)

        self.image_path = rospy.get_param("~image_path", "")
        self.image_topic = rospy.get_param("~image_topic", "/yaofang_test/image_raw")
        self.frame_id = rospy.get_param("~frame_id", "yaofang_test_camera")
        self.rate_hz = float(rospy.get_param("~rate", 10.0))

        if not self.image_path:
            rospy.logfatal("参数 ~image_path 未设置")
            raise rospy.ROSInitException("image_path is required")

        if not os.path.isfile(self.image_path):
            rospy.logfatal("图片不存在: %s", self.image_path)
            raise rospy.ROSInitException("image file not found")

        self._bgr = cv2.imread(self.image_path, cv2.IMREAD_COLOR)
        if self._bgr is None:
            rospy.logfatal("OpenCV 无法读取: %s", self.image_path)
            raise rospy.ROSInitException("failed to read image")

        self._bridge = CvBridge()
        self._pub = rospy.Publisher(self.image_topic, Image, queue_size=1)
        self._seq = 0

        rospy.loginfo(
            "测试图像发布: topic=%s path=%s size=%dx%d rate=%.1fHz",
            self.image_topic,
            self.image_path,
            self._bgr.shape[1],
            self._bgr.shape[0],
            self.rate_hz,
        )

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            msg = self._bridge.cv2_to_imgmsg(self._bgr, encoding="bgr8")
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.header.seq = self._seq
            self._seq += 1
            self._pub.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = TestImagePublisher()
        node.spin()
    except rospy.ROSInterruptException:
        pass
