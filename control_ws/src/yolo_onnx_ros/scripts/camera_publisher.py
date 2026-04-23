#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraPublisher:
    def __init__(self):
        rospy.init_node('camera_publisher', anonymous=True)
        
        # 参数
        self.camera_index = rospy.get_param('~camera_index', 0)
        self.frame_rate = rospy.get_param('~frame_rate', 30)
        self.resolution = rospy.get_param('~resolution', [640, 480])
        
        # 设置发布者
        self.image_pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
        self.bridge = CvBridge()
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        if not self.cap.isOpened():
            rospy.logerr("无法打开摄像头")
            return
            
        rospy.loginfo(f"摄像头发布节点已启动，索引: {self.camera_index}")

    def run(self):
        rate = rospy.Rate(self.frame_rate)
        
        while not rospy.is_shutdown() and self.cap.isOpened():
            ret, frame = self.cap.read()
            
            if ret:
                try:
                    # 转换为ROS Image消息并发布
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    ros_image.header.stamp = rospy.Time.now()
                    self.image_pub.publish(ros_image)
                except Exception as e:
                    rospy.logerr(f"转换图像时出错: {e}")
            
            rate.sleep()
        
        self.cap.release()

if __name__ == '__main__':
    try:
        node = CameraPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass