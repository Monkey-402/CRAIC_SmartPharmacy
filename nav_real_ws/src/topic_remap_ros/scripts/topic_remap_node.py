#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Relay topics so craic nodes (e.g. /camera/image_raw, /scan) match real robot_ws names."""

import copy
import rospy
from sensor_msgs.msg import Image, LaserScan


def main():
    rospy.init_node("topic_remap_node")

    relay_rgb = rospy.get_param("~relay_rgb", True)
    rgb_in = rospy.get_param("~rgb_in", "/camera/rgb/image_raw")
    rgb_out = rospy.get_param("~rgb_out", "/camera/image_raw")
    rgb_queue = rospy.get_param("~rgb_queue_size", 2)

    relay_scan = rospy.get_param("~relay_scan", True)
    scan_in = rospy.get_param("~scan_in", "/scan_filtered")
    scan_out = rospy.get_param("~scan_out", "/scan")
    scan_queue = rospy.get_param("~scan_queue_size", 5)
    scan_frame_id = rospy.get_param("~scan_frame_id", "laser_link")

    pub_rgb = None
    pub_scan = None

    def cb_rgb(msg):
        pub_rgb.publish(msg)

    def cb_scan(msg):
        out = copy.deepcopy(msg)
        if scan_frame_id:
            out.header.frame_id = scan_frame_id
        pub_scan.publish(out)

    subs = []
    if relay_rgb:
        pub_rgb = rospy.Publisher(rgb_out, Image, queue_size=rgb_queue, latch=False)
        subs.append(rospy.Subscriber(rgb_in, Image, cb_rgb, queue_size=rgb_queue))
        rospy.loginfo("topic_remap: %s -> %s (Image)", rgb_in, rgb_out)
    if relay_scan:
        pub_scan = rospy.Publisher(scan_out, LaserScan, queue_size=scan_queue, latch=False)
        subs.append(rospy.Subscriber(scan_in, LaserScan, cb_scan, queue_size=scan_queue))
        rospy.loginfo(
            "topic_remap: %s -> %s (LaserScan, frame_id=%s)",
            scan_in,
            scan_out,
            scan_frame_id or "(unchanged)",
        )

    if not subs:
        rospy.logwarn("topic_remap: all relays disabled; exiting")
        return

    rospy.sleep(0.2)
    rospy.spin()


if __name__ == "__main__":
    main()
