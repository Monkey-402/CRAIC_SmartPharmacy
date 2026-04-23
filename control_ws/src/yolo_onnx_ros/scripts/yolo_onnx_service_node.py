#!/usr/bin/env python3

import os
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import onnxruntime as ort
from yolo_onnx_ros.srv import ObjectDetection, ObjectDetectionResponse
from collections import Counter

class YOLOv8DetectorService:
    def __init__(self):
        self.bridge = CvBridge()
        
        # 参数配置
        self.model_path = rospy.get_param('~model_path', './models/best.onnx')
        self.conf_threshold = rospy.get_param('~conf_threshold', 0.5)
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.4)
        
        # 类别映射
        self.class_names = {
            0: "peopleInside",
            1: "peopleOutside",
            2: "object3",
            3: "object4",
            4: "object5"
        }
        self.nc = 5

        # 保存配置
        self.save_dir = '/home/zinn/snapshots'#修改
        self.save_idx = 0
        os.makedirs(self.save_dir, exist_ok=True)

        # 检查模型文件
        if not os.path.isfile(self.model_path):
            rospy.logerr(f"模型文件未找到: {self.model_path}")
            rospy.signal_shutdown("模型文件未找到")
            return
        
        try:
            self.session = ort.InferenceSession(
                self.model_path, 
                providers=['CPUExecutionProvider']
            )
            rospy.loginfo("模型加载成功")
        except Exception as e:
            rospy.logerr(f"模型加载失败: {str(e)}")
            rospy.signal_shutdown("模型加载失败")
            return
        
        # 获取模型信息
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [output.name for output in self.session.get_outputs()]
        
        self.input_shape = self.session.get_inputs()[0].shape
        self.input_height, self.input_width = self.input_shape[2:4]
        
        # 创建服务
        self.service = rospy.Service('~/detect_objects', ObjectDetection, self.handle_detection_request)
        
        rospy.loginfo(f"YOLOv8检测器服务初始化完成")
        rospy.loginfo(f"输入尺寸: {self.input_width}x{self.input_height}")
    
    def handle_detection_request(self, req):
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(req.image, desired_encoding='bgr8')
            
            # 推理
            input_tensor = self.preprocess(cv_image)
            outputs = self.session.run(self.output_names, {self.input_name: input_tensor})
            detections = self.postprocess(outputs[0], cv_image.shape)
            
            # 统计结果
            class_counts = self.count_objects(detections)
            
            # 构建响应
            response = ObjectDetectionResponse()
            
            for detection in detections:
                response.detected_classes.append(detection['class_name'])
                response.confidences.append(detection['score'])
                response.bboxes.append(detection['bbox'][0])
                response.bboxes.append(detection['bbox'][1])
                response.bboxes.append(detection['bbox'][2])
                response.bboxes.append(detection['bbox'][3])
            
            for class_name, count in class_counts.items():
                response.class_names.append(class_name)
                response.class_counts.append(count)
            
            response.success = True
            response.message = f"检测到 {len(detections)} 个物体"
            
            rospy.loginfo(f"服务处理完成: {response.message}")
            
            # 检测到人员时保存图像
            if any((d['class_id'] == 1 or d['class_id'] == 0) for d in detections):
                vis = cv_image.copy()
                for d in detections:
                    if d['class_id'] in [0, 1]:
                        color = (0, 255, 0) if d['class_id'] == 0 else (0, 0, 255)
                        x1, y1, x2, y2 = d['bbox']
                        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
                        label = f"{d['class_name']}:{d['score']:.2f}"
                        cv2.putText(vis, label, (x1, y1-5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
                save_path = os.path.join(self.save_dir, f"people{self.save_idx:02d}.jpg")
                cv2.imwrite(save_path, vis)
                rospy.loginfo(f"保存人员检测图: {save_path}")
                self.save_idx += 1
                
            return response
            
        except Exception as e:
            rospy.logerr(f"处理检测请求时出错: {str(e)}")
            response = ObjectDetectionResponse()
            response.success = False
            response.message = f"处理失败: {str(e)}"
            return response
    
    def count_objects(self, detections):
        class_names = [detection['class_name'] for detection in detections]
        return dict(Counter(class_names))
    
    def preprocess(self, image):
        input_img = cv2.resize(image, (self.input_width, self.input_height))
        input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
        input_img = input_img.transpose(2, 0, 1)
        input_tensor = input_img[np.newaxis, :, :, :].astype(np.float32) / 255.0
        return input_tensor
    
    def postprocess(self, output, orig_shape):
        outputs = np.transpose(output, (0, 2, 1))
        
        boxes = []
        scores = []
        class_ids = []
        
        orig_h, orig_w = orig_shape[:2]
        input_h, input_w = self.input_height, self.input_width
        
        for i in range(outputs.shape[1]):
            detection = outputs[0, i, :]
            classes_scores = detection[4:4+self.nc]
            max_score = np.max(classes_scores)
            class_id = np.argmax(classes_scores)
            
            if max_score >= self.conf_threshold:
                cx, cy, w, h = detection[0], detection[1], detection[2], detection[3]
                x1 = int((cx - w / 2) * orig_w / input_w)
                y1 = int((cy - h / 2) * orig_h / input_h)
                x2 = int((cx + w / 2) * orig_w / input_w)
                y2 = int((cy + h / 2) * orig_h / input_h)
                
                x1 = max(0, min(x1, orig_w - 1))
                y1 = max(0, min(y1, orig_h - 1))
                x2 = max(0, min(x2, orig_w - 1))
                y2 = max(0, min(y2, orig_h - 1))
                
                bbox_width = x2 - x1
                bbox_height = y2 - y1
                
                if bbox_width > 5 and bbox_height > 5:
                    boxes.append([x1, y1, x2, y2])
                    scores.append(max_score)
                    class_ids.append(class_id)
        
        if len(boxes) > 0:
            boxes_xywh = []
            for box in boxes:
                x1, y1, x2, y2 = box
                boxes_xywh.append([x1, y1, x2 - x1, y2 - y1])
            
            indices = cv2.dnn.NMSBoxes(boxes_xywh, scores, self.conf_threshold, self.iou_threshold)
            
            final_boxes = []
            final_scores = []
            final_class_ids = []
            
            if len(indices) > 0:
                for i in indices.flatten():
                    final_boxes.append(boxes[i])
                    final_scores.append(scores[i])
                    final_class_ids.append(class_ids[i])
            
            detections = []
            for i in range(len(final_boxes)):
                detections.append({
                    'class_id': final_class_ids[i],
                    'class_name': self.class_names.get(final_class_ids[i], "unknown"),
                    'score': final_scores[i],
                    'bbox': final_boxes[i]
                })
            
            return detections
        
        return []

def main():
    rospy.init_node('yolov8_detector_service', anonymous=True)
    
    try:
        detector_service = YOLOv8DetectorService()
        rospy.loginfo("YOLOv8检测器服务节点已启动")
        rospy.loginfo("等待检测请求...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被用户中断")
    except Exception as e:
        rospy.logerr(f"节点启动失败: {str(e)}")

if __name__ == '__main__':
    main()