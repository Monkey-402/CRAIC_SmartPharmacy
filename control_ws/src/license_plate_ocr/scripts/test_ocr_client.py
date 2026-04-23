#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from license_plate_ocr.srv import LicensePlateOCR
from sensor_msgs.msg import Image

def test_ocr_client(image_path):
    rospy.wait_for_service('license_plate_ocr')
    
    try:
        # 读取测试图片
        cv_image = cv2.imread(image_path)
        if cv_image is None:
            rospy.logerr("无法读取图片")
            return
        
        # 转换图片格式
        bridge = CvBridge()
        image_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        
        # 调用服务
        ocr_service = rospy.ServiceProxy('license_plate_ocr', LicensePlateOCR)
        response = ocr_service(image_msg)
        
        # 输出结果
        print(f"状态: {response.status_message}")
        for i, (text, conf) in enumerate(zip(response.plate_texts, response.confidences)):
            print(f"车牌 {i+1}: {text}, 置信度: {conf:.4f}")
            
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")

if __name__ == "__main__":
    rospy.init_node('test_ocr_client')
    test_ocr_client("/home/chen/gazebo_ws/src/license_plate_ocr/cp.png")  # 替换为您的测试图片路径