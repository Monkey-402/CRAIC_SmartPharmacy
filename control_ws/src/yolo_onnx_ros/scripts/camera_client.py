#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolo_onnx_ros.srv import ObjectDetection, ObjectDetectionRequest

class CameraDetectionClient:
    def __init__(self):
        rospy.init_node('camera_detection_client', anonymous=True)
        
        # 参数
        self.camera_index = rospy.get_param('~camera_index', 0)
        self.frame_rate = rospy.get_param('~frame_rate', 10)
        self.resolution = rospy.get_param('~resolution', [640, 480])
        self.detection_service_name = rospy.get_param('~detection_service', 'detect_objects')
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 等待检测服务可用
        rospy.loginfo(f"等待检测服务: {self.detection_service_name}")
        rospy.wait_for_service(self.detection_service_name)
        self.detection_service = rospy.ServiceProxy(self.detection_service_name, ObjectDetection)
        rospy.loginfo("检测服务已连接")
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        if not self.cap.isOpened():
            rospy.logerr("无法打开摄像头")
            return
            
        rospy.loginfo(f"摄像头检测客户端已启动，索引: {self.camera_index}")

    def process_frame(self, frame):
        """处理单帧图像并调用检测服务"""
        try:
            # 将OpenCV图像转换为ROS Image消息
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.stamp = rospy.Time.now()
            
            # 创建服务请求
            request = ObjectDetectionRequest()
            request.image = ros_image
            
            # 调用检测服务
            response = self.detection_service(request)
            
            # 处理检测结果
            if response.success:
                self.log_results(response)
            else:
                rospy.logwarn(f"检测失败: {response.message}")
                
            return response
            
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")
            return None
        except Exception as e:
            rospy.logerr(f"处理图像时出错: {e}")
            return None

    def log_results(self, response):
        """记录检测结果到日志"""
        if response.detected_classes:
            # 输出检测摘要
            rospy.loginfo("=" * 50)
            rospy.loginfo(f"检测结果: {response.message}")
            
            # 输出每个检测到的物体详细信息
            bbox_index = 0
            for i, class_name in enumerate(response.detected_classes):
                if bbox_index + 3 < len(response.bboxes):
                    x1 = response.bboxes[bbox_index]
                    y1 = response.bboxes[bbox_index + 1]
                    x2 = response.bboxes[bbox_index + 2]
                    y2 = response.bboxes[bbox_index + 3]
                    bbox_index += 4
                    
                    confidence = response.confidences[i]
                    width = x2 - x1
                    height = y2 - y1
                    
                    rospy.loginfo(f"  {i+1}. {class_name}: {confidence:.3f} "
                                 f"位置: [{x1}, {y1}, {x2}, {y2}] "
                                 f"尺寸: {width}x{height}")
            
            # 输出类别统计信息
            if response.class_names and response.class_counts:
                rospy.loginfo("-" * 30)
                rospy.loginfo("类别统计:")
                total_count = 0
                for name, count in zip(response.class_names, response.class_counts):
                    rospy.loginfo(f"  {name}: {count}个")
                    total_count += count
                rospy.loginfo(f"总计: {total_count}个物体")
            rospy.loginfo("=" * 50)
        else:
            rospy.loginfo("未检测到任何物体")

    def run(self):
        """主循环"""
        rate = rospy.Rate(self.frame_rate)
        
        while not rospy.is_shutdown() and self.cap.isOpened():
            ret, frame = self.cap.read()
            
            if ret:
                # 处理当前帧
                self.process_frame(frame)
            
            # 检查是否按下Ctrl+C退出
            if rospy.is_shutdown():
                break
                
            rate.sleep()
        
        # 清理资源
        self.cap.release()
        rospy.loginfo("摄像头检测客户端已关闭")

if __name__ == '__main__':
    try:
        client = CameraDetectionClient()
        client.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被用户中断")
    except Exception as e:
        rospy.logerr(f"客户端运行错误: {e}")