#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import pytesseract
import sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

reload(sys)
sys.setdefaultencoding('utf-8')

class OCRNode:
    def __init__(self):
        rospy.init_node('ocr_node', anonymous=True)
        
        # 摄像头话题
        self.image_topic = '/camera/image_raw'
        
        # 订阅摄像头图像
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        
        # 发布识别结果
        self.result_pub = rospy.Publisher('/ocr/result', String, queue_size=10)
        
        self.bridge = CvBridge()
        self.rate = rospy.Rate(1)
        self.latest_image = None
        
        rospy.loginfo("OCR node started")
        rospy.loginfo("Subscribe topic: %s" % self.image_topic)
        rospy.loginfo("Publish topic: /ocr/result")
        
    def image_callback(self, msg):
        self.latest_image = msg
        
    def run(self):
        while not rospy.is_shutdown():
            if self.latest_image is not None:
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
                    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                    
                    # OCR 识别
                    text_raw = pytesseract.image_to_string(gray, lang='chi_sim+eng')
                    
                    # 清理文本（去除空白和特殊字符）
                    text = text_raw.strip()
                    
                    if text:
                        result_msg = String()
                        result_msg.data = text
                        self.result_pub.publish(result_msg)
                        
                        # 安全打印中文
                        try:
                            rospy.loginfo("Recognized: " + text.encode('utf-8'))
                        except:
                            rospy.loginfo("Recognized: (Chinese text)")
                        
                except Exception as e:
                    rospy.logerr("OCR error: %s" % str(e))
                    
            self.rate.sleep()

if __name__ == '__main__':
    try:
        ocr = OCRNode()
        ocr.run()
    except rospy.ROSInterruptException:
        pass
