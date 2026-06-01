#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import re
import sys
import unicodedata

import cv2
import numpy as np
import pytesseract
import rospy

from move_nav.srv import Board2Decode, Board2DecodeResponse

from ocr_frame_detector import (
    A4FrameParams,
    apply_dark_mask,
    crop_a4_frame,
    draw_frame_box,
)

try:
    text_type = unicode
    binary_type = str
except NameError:
    text_type = str
    binary_type = bytes


class OCRService(object):
    def __init__(self):
        rospy.init_node("ocr_service", anonymous=True)

        service_name = rospy.get_param(
            "~board2_decode_service",
            "/yaofang_vision/board2_decode",
        )
        self.dark_mask_threshold = int(rospy.get_param("~dark_mask_threshold", 120))
        self.roi_x1_ratio = float(rospy.get_param("~roi_x1_ratio", 0.08))
        self.roi_y1_ratio = float(rospy.get_param("~roi_y1_ratio", 0.18))
        self.roi_x2_ratio = float(rospy.get_param("~roi_x2_ratio", 0.92))
        self.roi_y2_ratio = float(rospy.get_param("~roi_y2_ratio", 0.88))
        self.tesseract_psm = int(rospy.get_param("~tesseract_psm", 7))
        self.use_keyword_inference = bool(
            rospy.get_param("~use_keyword_inference", True)
        )
        self.use_frame_detect = bool(rospy.get_param("~use_frame_detect", True))
        self.frame_params = A4FrameParams.from_rosparam(rospy)
        self.frame_params.dark_mask_threshold = self.dark_mask_threshold
        self.save_debug_images = bool(rospy.get_param("~save_debug_images", True))

        self._warm_up_tesseract()
        self.service = rospy.Service(
            service_name,
            Board2Decode,
            self.handle_request,
        )
        rospy.loginfo("OCR board2 decode service started: %s", service_name)
        rospy.loginfo(
            "OCR config: dark_mask_thr=%d frame_detect=%s psm=%d",
            self.dark_mask_threshold,
            self.use_frame_detect,
            self.tesseract_psm,
        )

    def _warm_up_tesseract(self):
        rospy.loginfo("Loading Tesseract OCR engine...")
        dummy_img = np.zeros((100, 100), dtype=np.uint8)
        pytesseract.image_to_string(
            dummy_img, lang="chi_sim+eng", config="--psm %d" % self.tesseract_psm
        )
        rospy.loginfo("Tesseract OCR engine ready")

    def _to_text(self, value):
        if value is None:
            return ""
        if isinstance(value, text_type):
            return value
        if isinstance(value, binary_type):
            return value.decode("utf-8", "ignore")
        return text_type(value)

    def _to_ros_string(self, value):
        value = self._to_text(value)
        if sys.version_info[0] < 3 and isinstance(value, text_type):
            return value.encode("utf-8")
        return value

    def _to_log_string(self, value):
        value = self._to_text(value)
        if sys.version_info[0] < 3 and isinstance(value, text_type):
            return value.encode("utf-8")
        return value

    def _extract_roi_by_ratio(self, cv_image):
        height, width = cv_image.shape[:2]
        x1 = max(0, int(width * self.roi_x1_ratio))
        y1 = max(0, int(height * self.roi_y1_ratio))
        x2 = min(width, int(width * self.roi_x2_ratio))
        y2 = min(height, int(height * self.roi_y2_ratio))
        if x2 <= x1 + 8 or y2 <= y1 + 8:
            return cv_image, "ratio_fallback_full_image"
        return cv_image[y1:y2, x1:x2], "ratio_roi"

    def _extract_roi(self, cv_image):
        frame_result = None
        if self.use_frame_detect:
            frame_result, crop = crop_a4_frame(cv_image, self.frame_params)
            if frame_result.ok and crop is not None:
                rospy.loginfo(
                    "A4 frame detected: x=%d y=%d w=%d h=%d",
                    frame_result.rect[0],
                    frame_result.rect[1],
                    frame_result.rect[2],
                    frame_result.rect[3],
                )
                return crop, "a4_frame", frame_result

            rospy.logwarn(
                "[OCR] %s，回退比例 ROI",
                frame_result.error_message or "frame_detect_failed",
            )

        roi, mode = self._extract_roi_by_ratio(cv_image)
        return roi, mode, frame_result

    def _preprocess_for_ocr(self, cv_image, roi_mode):
        _masked_bgr, mask = apply_dark_mask(cv_image, self.dark_mask_threshold)
        mode = "dark_mask_%d" % self.dark_mask_threshold
        if roi_mode != "a4_frame":
            mode += "_ratio"
        return mask, mode

    def _run_tesseract(self, processed_image):
        config = "--psm %d" % self.tesseract_psm
        text_raw = pytesseract.image_to_string(
            processed_image,
            lang="chi_sim+eng",
            config=config,
        )
        return self._to_text(text_raw).strip()

    def _infer_board2_speech(self, raw_text):
        if not self.use_keyword_inference:
            return self._to_text(raw_text).strip()

        normalized = unicodedata.normalize("NFKC", self._to_text(raw_text))
        normalized = normalized.replace(" ", "")

        has_lab = (
            (u"化验" in normalized)
            or (u"化" in normalized and u"验" in normalized)
            or (u"化" in normalized)
        )
        has_idle = (
            (u"空闲" in normalized)
            or (u"空" in normalized and u"闲" in normalized)
            or (u"快速" in normalized)
        )
        has_busy = (
            (u"忙碌" in normalized)
            or (u"忙" in normalized)
            or (u"等待" in normalized)
        )

        if has_lab and has_idle:
            return u"化验区空闲中，请快速通过"
        if has_lab and has_busy:
            seconds = self.extract_wait_seconds(normalized)
            if seconds > 0:
                return u"化验区忙碌中，需等待 %d 秒" % seconds
            return u"化验区忙碌中，需等待"
        if has_idle:
            return u"化验区空闲中，请快速通过"
        if has_busy:
            seconds = self.extract_wait_seconds(normalized)
            if seconds > 0:
                return u"化验区忙碌中，需等待 %d 秒" % seconds

        return self._to_text(raw_text).strip()

    def _save_debug_images(self, image_path, cv_image, roi_bgr, processed, preprocess_mode, frame_result, roi_mode):
        if not self.save_debug_images or not image_path:
            return

        base, _ext = os.path.splitext(image_path)
        roi_path = base + "_ocr_roi.jpg"
        input_path = base + "_ocr_input.jpg"
        box_path = base + "_ocr_frame_box.jpg"
        mask_path = base + "_ocr_frame_mask.jpg"
        try:
            cv2.imwrite(roi_path, roi_bgr)
            cv2.imwrite(input_path, processed)
            if frame_result is not None and frame_result.rect is not None:
                boxed = draw_frame_box(cv_image, frame_result.rect)
                cv2.imwrite(box_path, boxed)
            if frame_result is not None and frame_result.dark_mask is not None:
                cv2.imwrite(mask_path, frame_result.dark_mask)
            rospy.loginfo(
                "OCR debug saved: roi=%s input=%s preprocess=%s roi_mode=%s",
                roi_path,
                input_path,
                preprocess_mode,
                roi_mode,
            )
        except Exception as exc:
            rospy.logwarn("Failed to save OCR debug images: %s", exc)

    def extract_wait_seconds(self, text):
        if not text:
            return 0

        normalized = unicodedata.normalize("NFKC", self._to_text(text))
        normalized = " ".join(normalized.split())

        if sys.version_info[0] < 3:
            regex_text = normalized.encode("utf-8")
            regex_pattern = "(\\d+)\\s*(?:\xe7\xa7\x92|s\\b|sec(?:ond)?s?\\b)"
        else:
            regex_text = normalized
            regex_pattern = u"(\\d+)\\s*(?:秒|s\\b|sec(?:ond)?s?\\b)"

        matches = re.findall(regex_pattern, regex_text, re.IGNORECASE)
        if not matches:
            return 0

        seconds = int(matches[0])
        rospy.loginfo("Extracted wait seconds: %d", seconds)
        return seconds

    def handle_request(self, req):
        wait_seconds = 0
        speech_text = ""

        if not req.image_path or not os.path.exists(req.image_path):
            rospy.logwarn("Image does not exist: %s", req.image_path)
            return Board2DecodeResponse(wait_seconds, speech_text)

        try:
            cv_image = cv2.imread(req.image_path)
            if cv_image is None:
                rospy.logerr("Failed to read image: %s", req.image_path)
                return Board2DecodeResponse(wait_seconds, speech_text)

            roi_bgr, roi_mode, frame_result = self._extract_roi(cv_image)
            processed, preprocess_mode = self._preprocess_for_ocr(roi_bgr, roi_mode)
            self._save_debug_images(
                req.image_path,
                cv_image,
                roi_bgr,
                processed,
                preprocess_mode,
                frame_result,
                roi_mode,
            )

            raw_text = self._run_tesseract(processed)
            speech_text = self._infer_board2_speech(raw_text)
            wait_seconds = self.extract_wait_seconds(speech_text)
            if wait_seconds == 0 and raw_text != speech_text:
                wait_seconds = self.extract_wait_seconds(raw_text)

            rospy.loginfo(
                "Board2 OCR result: wait_seconds=%d speech_text=%s (raw=%s, "
                "roi=%s preprocess=%s)",
                wait_seconds,
                self._to_log_string(speech_text),
                self._to_log_string(raw_text),
                roi_mode,
                preprocess_mode,
            )
        except Exception as exc:
            rospy.logerr("OCR failed: %s", exc)

        return Board2DecodeResponse(wait_seconds, self._to_ros_string(speech_text))


if __name__ == "__main__":
    try:
        OCRService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
