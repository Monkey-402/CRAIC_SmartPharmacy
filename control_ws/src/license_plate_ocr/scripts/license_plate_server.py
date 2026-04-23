#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import onnxruntime as ort
from cv_bridge import CvBridge
from license_plate_ocr.srv import LicensePlateOCR, LicensePlateOCRResponse
import os
import re
from paddleocr import PaddleOCR

class LicensePlateOCRServer:
    def __init__(self):
        rospy.init_node('license_plate_ocr_server')
        self.bridge = CvBridge()
        
        # 参数配置
        self.model_path = rospy.get_param('~model_path', 'models/best.onnx')
        self.conf_thres = rospy.get_param('~conf_thres', 0.4)
        self.iou_thres = rospy.get_param('~iou_thres', 0.5)
        self.save_dir = rospy.get_param('~save_dir', '/home/zinn/snapshots')#修改
        
        os.makedirs(self.save_dir, exist_ok=True)
        
        # 初始化模型
        self.init_yolo_model()
        self.init_paddle_ocr()
        
        self.service = rospy.Service('license_plate_ocr', LicensePlateOCR, self.handle_ocr_request)
        rospy.loginfo("License Plate OCR Server 已启动")
    
    def init_yolo_model(self):
        """初始化YOLO模型"""
        try:
            if not os.path.exists(self.model_path):
                rospy.logerr(f"模型文件不存在: {self.model_path}")
                return False
            
            self.session = ort.InferenceSession(self.model_path, providers=["CPUExecutionProvider"])
            self.input_name = self.session.get_inputs()[0].name
            input_shape = self.session.get_inputs()[0].shape
            self.img_h, self.img_w = input_shape[2], input_shape[3]
            
            rospy.loginfo("YOLOv8模型加载成功")
            return True
            
        except Exception as e:
            rospy.logerr(f"YOLO模型初始化失败: {e}")
            return False
    
    def init_paddle_ocr(self):
        """初始化PaddleOCR"""
        try:
            self.ocr = PaddleOCR(
                use_angle_cls=False,
                lang='ch',
                use_gpu=False,
                det=True,
                rec=True,
                cls=False
            )
            rospy.loginfo("PaddleOCR初始化成功")
        except Exception as e:
            rospy.logerr(f"PaddleOCR初始化失败: {e}")
    
    def preprocess(self, image):
        """图像预处理"""
        image_resized = cv2.resize(image, (self.img_w, self.img_h))
        image_rgb = cv2.cvtColor(image_resized, cv2.COLOR_BGR2RGB)
        image_norm = image_rgb.astype(np.float32) / 255.0
        image_transposed = np.transpose(image_norm, (2, 0, 1))
        return np.expand_dims(image_transposed, axis=0)
    
    def nms(self, boxes, scores, iou_thres):
        """非极大值抑制"""
        if len(boxes) == 0:
            return []

        boxes = np.array(boxes)
        scores = np.array(scores)
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h
            iou = inter / (areas[i] + areas[order[1:]] - inter)

            inds = np.where(iou <= iou_thres)[0]
            order = order[inds + 1]
        return keep
    
    def detect_objects(self, image):
        """YOLO目标检测"""
        h0, w0 = image.shape[:2]
        input_tensor = self.preprocess(image)

        preds = self.session.run(None, {self.input_name: input_tensor})[0]
        preds = preds[0].T

        boxes, scores = [], []
        for x, y, w, h, conf in preds:
            if conf < self.conf_thres:
                continue
            x1 = int((x - w / 2) * w0 / self.img_w)
            y1 = int((y - h / 2) * h0 / self.img_h)
            x2 = int((x + w / 2) * w0 / self.img_w)
            y2 = int((y + h / 2) * h0 / self.img_h)
            boxes.append([x1, y1, x2, y2])
            scores.append(conf)

        keep = self.nms(boxes, scores, self.iou_thres)
        boxes = [boxes[i] for i in keep]
        scores = [scores[i] for i in keep]
        return boxes, scores
    
    def draw_detection_results(self, image, boxes, scores, plate_texts):
        """绘制检测结果"""
        result_image = image.copy()
        
        colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255)]
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        for i, (box, score, text) in enumerate(zip(boxes, scores, plate_texts)):
            x1, y1, x2, y2 = box
            color = colors[i % len(colors)]
            
            # 绘制边界框
            cv2.rectangle(result_image, (x1, y1), (x2, y2), color, 2)
            
            label = f"{text} ({score:.2f})"
            (text_width, text_height), baseline = cv2.getTextSize(label, font, 0.7, 2)
            
            # 绘制文本背景和文字
            cv2.rectangle(result_image, 
                         (x1, y1 - text_height - baseline - 5), 
                         (x1 + text_width, y1), 
                         color, -1)
            cv2.putText(result_image, label, (x1, y1 - baseline - 2), 
                       font, 0.7, (255, 255, 255), 2)
        
        return result_image
    
    def save_result_image(self, image):
        """保存结果图像"""
        try:
            save_path = os.path.join(self.save_dir, "cp.jpg")
            if os.path.exists(save_path):
                os.remove(save_path)
            
            if cv2.imwrite(save_path, image):
                rospy.loginfo(f"图像已保存: {save_path}")
                return True
            return False
        except Exception as e:
            rospy.logerr(f"保存失败: {e}")
            return False
    
    def handle_ocr_request(self, req):
        """处理OCR请求"""
        rospy.loginfo("收到OCR识别请求")
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(req.image, "bgr8")
            original_image = cv_image.copy()
            
            # 检测车牌
            boxes, scores = self.detect_objects(cv_image)
            
            plate_texts = []
            confidences = []
            ocr_results = []
            
            for i, (x1, y1, x2, y2) in enumerate(boxes):
                crop = cv_image[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
                if crop.size == 0:
                    continue
                
                # OCR识别
                result = self.ocr.ocr(crop, cls=False)
                rospy.loginfo(f"完整OCR结果: {result}")

                if result and len(result) > 0:
                    # 递归或循环提取第一个有效识别结果
                    def extract_text_info(data):
                        """递归提取 (文本, 置信度) 元组"""
                        if isinstance(data, (list, tuple)):
                            if len(data) == 2 and isinstance(data[1], (list, tuple)) and len(data[1]) == 2 and isinstance(data[1][0], str):
                                # 格式: [坐标, (文本, 置信度)]
                                return data[1]
                            for item in data:
                                res = extract_text_info(item)
                                if res:
                                    return res
                        return None
                    
                    text_info = extract_text_info(result)
                    
                    if text_info and len(text_info) >= 2:
                        detected_text = str(text_info[0])
                        confidence = float(text_info[1])
                        processed_text = self.postprocess_plate(detected_text)
                        
                        plate_texts.append(processed_text)
                        confidences.append(confidence)
                        ocr_results.append(processed_text)
                        
                        rospy.loginfo(f"成功识别车牌: {processed_text}, 置信度: {confidence:.4f}")
                    else:
                        rospy.logwarn(f"无法从OCR结果中提取文本: {result}")
                else:
                    rospy.logwarn(f"OCR无结果或结果为空")
            
            # 绘制并保存结果
            if boxes and ocr_results:
                result_image = self.draw_detection_results(original_image, boxes, scores, ocr_results)
                self.save_result_image(result_image)
                rospy.loginfo(f"识别到车牌: {ocr_results[0]}")
            else:
                self.save_result_image(original_image)
                rospy.loginfo("未检测到车牌")
            
            return LicensePlateOCRResponse(
                plate_texts=plate_texts,
                confidences=confidences,
                status_message="识别成功" if plate_texts else "未识别到车牌"
            )
                
        except Exception as e:
            rospy.logerr(f"处理失败: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return LicensePlateOCRResponse(
                plate_texts=[],
                confidences=[],
                status_message=f"处理失败: {str(e)}"
            )
    
    def postprocess_plate(self, text):
        """车牌文本后处理"""
        text = text.replace(" ", "").replace("·", "").replace(":", "")
        match = re.search(r'[\u4e00-\u9fff][A-Z0-9]{5,7}', text)
        return match.group(0) if match else text

def main():
    server = LicensePlateOCRServer()
    rospy.spin()

if __name__ == "__main__":
    main()