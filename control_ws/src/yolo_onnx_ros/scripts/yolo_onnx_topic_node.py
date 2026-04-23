#!/usr/bin/env python3

import os
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import onnxruntime as ort

class YOLOv8Detector:
    """YOLOv8物体检测器类，用于在ROS环境中进行实时目标检测"""
    
    def __init__(self):
        """初始化YOLOv8检测器，加载模型并设置ROS话题"""
        # 初始化CV桥接器，用于ROS图像消息和OpenCV图像之间的转换
        self.bridge = CvBridge()
        
        # 从ROS参数服务器获取配置参数
        self.model_path = rospy.get_param('~model_path', '/path/to/your/yolov8.onnx')  # ONNX模型文件路径
        self.input_topic = rospy.get_param('~input_topic', '/camera/image_raw')        # 输入图像话题
        self.conf_threshold = rospy.get_param('~conf_threshold', 0.5)                  # 置信度阈值
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.4)                    # IOU阈值
        
        # 定义类别名称映射表，将类别ID映射到可读名称
        self.class_names = {
            0: "object1",   # 类别1
            1: "object2",   # 类别2
            2: "object3",   # 类别3
            3: "object4",   # 类别4
            4: "object5"    # 类别5
        }
        self.nc = 5  # 类别数量
        
        # 检查模型文件是否存在
        if not os.path.isfile(self.model_path):
            rospy.logerr(f"模型文件未找到: {self.model_path}")
            rospy.logerr("请使用参数 'model_path' 指定正确的模型路径")
            rospy.signal_shutdown("模型文件未找到")
            return
        
        try:
            # 创建ONNX运行时会话，指定使用CPU进行推理
            self.session = ort.InferenceSession(
                self.model_path, 
                providers=['CPUExecutionProvider']  # 使用CPU执行提供者
            )
            rospy.loginfo("模型加载成功")
        except Exception as e:
            rospy.logerr(f"模型加载失败: {str(e)}")
            rospy.signal_shutdown("模型加载失败")
            return
        
        # 获取模型输入输出信息
        self.input_name = self.session.get_inputs()[0].name        # 输入节点名称
        self.output_names = [output.name for output in self.session.get_outputs()]  # 输出节点名称列表
        
        # 获取输入张量的形状信息
        self.input_shape = self.session.get_inputs()[0].shape
        self.input_height, self.input_width = self.input_shape[2:4]  # 输入图像的高度和宽度
        
        # 创建发布器，用于发布带检测结果的图像
        self.image_pub = rospy.Publisher('~/detections_image', Image, queue_size=1)
        
        # 创建订阅器，订阅输入图像话题
        self.image_sub = rospy.Subscriber(
            self.input_topic, 
            Image, 
            self.image_callback, 
            queue_size=1, 
            buff_size=2**24  # 设置较大的缓冲区大小以处理高分辨率图像
        )
        
        # 输出初始化完成信息
        rospy.loginfo(f"YOLOv8检测器初始化完成")
        rospy.loginfo(f"输入话题: {self.input_topic}")
        rospy.loginfo(f"模型路径: {self.model_path}")
        rospy.loginfo(f"输入尺寸: {self.input_width}x{self.input_height}")
        rospy.loginfo(f"输出话题: /yolov8_detector/detections_image")
        
    def image_callback(self, msg):
        """图像回调函数，处理接收到的图像消息"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            original_image = cv_image.copy()  # 保存原始图像副本
            
            # 预处理图像，准备输入张量
            input_tensor = self.preprocess(cv_image)
            
            # 运行模型推理
            outputs = self.session.run(self.output_names, {self.input_name: input_tensor})
            
            # 后处理模型输出，提取检测结果
            detections = self.postprocess(outputs[0], original_image.shape)
            
            # 在图像上可视化检测结果
            result_image = self.visualize(original_image, detections)
            
            # 将处理后的图像转换回ROS消息格式并发布
            detection_image_msg = self.bridge.cv2_to_imgmsg(result_image, encoding="bgr8")
            detection_image_msg.header = msg.header  # 保持原始消息头信息
            self.image_pub.publish(detection_image_msg)
            
            # 在终端输出检测结果信息（可选）
            if detections:
                rospy.loginfo(f"检测到 {len(detections)} 个物体")
                for detection in detections:
                    rospy.loginfo(f"  - {detection['class_name']}: {detection['score']:.2f} "
                                 f"位置: [{detection['bbox'][0]}, {detection['bbox'][1]}, "
                                 f"{detection['bbox'][2]}, {detection['bbox'][3]}]")
            
        except Exception as e:
            rospy.logerr(f"处理图像时出错: {str(e)}")
    
    def preprocess(self, image):
        """预处理图像，调整为模型输入尺寸并归一化"""
        # 调整图像大小到模型输入尺寸
        input_img = cv2.resize(image, (self.input_width, self.input_height))
        
        # 转换颜色通道从BGR到RGB
        input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
        
        # 转换图像维度从HWC（高度、宽度、通道）到CHW（通道、高度、宽度）
        input_img = input_img.transpose(2, 0, 1)
        
        # 添加批次维度并归一化像素值到[0,1]范围
        input_tensor = input_img[np.newaxis, :, :, :].astype(np.float32) / 255.0
        
        return input_tensor
    
    def postprocess(self, output, orig_shape):
        """后处理模型输出，提取边界框并应用非极大值抑制"""
        # 转置输出张量维度
        outputs = np.transpose(output, (0, 2, 1))
        
        boxes = []       # 存储边界框坐标
        scores = []      # 存储置信度分数
        class_ids = []   # 存储类别ID
        
        # 获取原始图像和输入图像的尺寸
        orig_h, orig_w = orig_shape[:2]
        input_h, input_w = self.input_height, self.input_width
        
        # 遍历所有检测结果
        for i in range(outputs.shape[1]):
            detection = outputs[0, i, :]  # 获取单个检测结果
            
            # 提取类别分数（跳过前4个边界框值）
            classes_scores = detection[4:4+self.nc]
            
            # 获取最高分数和对应的类别ID
            max_score = np.max(classes_scores)
            class_id = np.argmax(classes_scores)
            
            # 过滤低置信度检测
            if max_score >= self.conf_threshold:
                # 提取边界框坐标 (中心点x, 中心点y, 宽度, 高度)
                cx, cy, w, h = detection[0], detection[1], detection[2], detection[3]
                
                # 将归一化坐标转换回原始图像坐标
                x1 = int((cx - w / 2) * orig_w / input_w)
                y1 = int((cy - h / 2) * orig_h / input_h)
                x2 = int((cx + w / 2) * orig_w / input_w)
                y2 = int((cy + h / 2) * orig_h / input_h)
                
                # 确保坐标在图像范围内
                x1 = max(0, min(x1, orig_w - 1))
                y1 = max(0, min(y1, orig_h - 1))
                x2 = max(0, min(x2, orig_w - 1))
                y2 = max(0, min(y2, orig_h - 1))
                
                # 计算边界框宽度和高度
                bbox_width = x2 - x1
                bbox_height = y2 - y1
                
                # 过滤掉太小的检测框（可能是噪声）
                if bbox_width > 5 and bbox_height > 5:
                    boxes.append([x1, y1, x2, y2])
                    scores.append(max_score)
                    class_ids.append(class_id)
        
        # 如果有检测结果，应用非极大值抑制
        if len(boxes) > 0:
            # 将边界框转换为(x, y, w, h)格式供NMS使用
            boxes_xywh = []
            for box in boxes:
                x1, y1, x2, y2 = box
                boxes_xywh.append([x1, y1, x2 - x1, y2 - y1])
            
            # 应用非极大值抑制，去除重叠的检测框
            indices = cv2.dnn.NMSBoxes(boxes_xywh, scores, self.conf_threshold, self.iou_threshold)
            
            final_boxes = []       # 最终边界框
            final_scores = []      # 最终置信度分数
            final_class_ids = []   # 最终类别ID
            
            # 提取NMS后的检测结果
            if len(indices) > 0:
                for i in indices.flatten():
                    final_boxes.append(boxes[i])
                    final_scores.append(scores[i])
                    final_class_ids.append(class_ids[i])
            
            # 组合检测结果信息
            detections = []
            for i in range(len(final_boxes)):
                detections.append({
                    'class_id': final_class_ids[i],  # 类别ID
                    'class_name': self.class_names.get(final_class_ids[i], "unknown"),  # 类别名称
                    'score': final_scores[i],        # 置信度分数
                    'bbox': final_boxes[i]           # 边界框坐标[x1, y1, x2, y2]
                })
            
            return detections
        
        return []  # 如果没有检测结果，返回空列表
    
    def visualize(self, image, detections):
        """在图像上可视化检测结果"""
        result_image = image.copy()  # 创建图像副本
        
        # 为每个检测结果绘制边界框和标签
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']  # 边界框坐标
            class_id = detection['class_id']     # 类别ID
            score = detection['score']           # 置信度分数
            class_name = detection['class_name'] # 类别名称
            
            # 获取类别对应的颜色
            color = self.get_color(class_id)
            
            # 绘制边界框
            cv2.rectangle(result_image, (x1, y1), (x2, y2), color, 2)
            
            # 创建标签文本（类别名称:置信度）
            label = f"{class_name}: {score:.2f}"
            
            # 计算文本尺寸
            (label_width, label_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            
            # 绘制标签背景矩形
            cv2.rectangle(
                result_image, 
                (x1, y1 - label_height - baseline), 
                (x1 + label_width, y1), 
                color, 
                -1  # 填充矩形
            )
            
            # 绘制文本标签
            cv2.putText(
                result_image, 
                label, 
                (x1, y1 - baseline), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.5, 
                (0, 0, 0),  # 黑色文本
                1           # 线宽
            )
        
        return result_image
    
    def get_color(self, class_id):
        """为不同类别生成不同颜色"""
        colors = [
            (255, 0, 0),    # 红色 - object1
            (0, 255, 0),    # 绿色 - object2
            (0, 0, 255),    # 蓝色 - object3
            (255, 255, 0),  # 青色 - object4
            (255, 0, 255)   # 紫色 - object5
        ]
        return colors[class_id % len(colors)]  # 使用取模确保不越界

def main():
    """主函数，初始化ROS节点并启动检测器"""
    # 初始化ROS节点
    rospy.init_node('yolov8_detector', anonymous=True)
    
    try:
        # 创建YOLOv8检测器实例
        detector = YOLOv8Detector()
        rospy.loginfo("YOLOv8检测器节点已启动")
        rospy.loginfo("正在等待图像输入...")
        
        # 进入ROS循环
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被用户中断")
    except Exception as e:
        rospy.logerr(f"节点启动失败: {str(e)}")

if __name__ == '__main__':
    main()