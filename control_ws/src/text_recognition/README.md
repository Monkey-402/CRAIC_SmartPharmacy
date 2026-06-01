# text_recognition

识别板二文字识别节点，按 `move_nav/Board2Decode` service 接口提供结果。

## 规则

识别板二是一张屏幕截图，节点读取图片路径后使用 Tesseract OCR 识别
中文和英文文本，并返回：

- `speech_text`：识别到的完整文字
- `wait_seconds`：从文字中提取出的等待秒数

等待秒数只匹配带时间单位的数字，例如 `30 秒`、`45s`、`60 seconds`。
这样可以避免把窗口号、取药口编号、日期或编号误判为等待时间。

识别板二是一张带 **A4 比例（≈1.414:1）黑框** 的告示/屏幕。流程：

1. **A4 黑框检测**（`ocr_frame_detector.py`，默认开启）→ 向内裁剪
2. 失败则回退 **比例 ROI**
3. **预处理**：框检出成功 → 灰度（不二值化）；回退 ROI → `dark` 高阈值二值化（只保留较黑像素）
4. 单次 Tesseract `--psm 7` + 关键词推断

调试图（与 snapshot 同目录）：

- `{N}_ocr_frame_box.jpg`：原图标出检测框
- `{N}_ocr_roi.jpg`：裁剪 ROI
- `{N}_ocr_input.jpg`：送入 Tesseract 的图像（灰度或二值化）

**OCR 仍不准？** 见 [OCR_ALTERNATIVES.md](OCR_ALTERNATIVES.md)（PaddleOCR、纯关键词等方案对比）。

## 安装依赖

apt-get update
apt-get install tesseract-ocr tesseract-ocr-chi-sim python-opencv python-pip -y
pip2 install 'pytesseract==0.2.9'

## 运行

```bash
cd CRAIC_SmartPharmacy/control_ws
catkin_make
source devel/setup.bash
roslaunch text_recognition ocr_service.launch
```

默认服务名：`/yaofang_vision/board2_decode`。

可选参数：

- `board2_decode_service`：文字识别服务名，默认 `/yaofang_vision/board2_decode`。
- `threshold_method`：回退 ROI 时的二值化方式，默认 **`dark`**（高阈值保留黑字），可改为 `adaptive` / `otsu` / `gray`。
- `dark_threshold`：`dark` 模式阈值，默认 **`145`**（越高越只保留较黑像素）。
- `gray_on_frame_detect`：A4 框检出成功时是否直接灰度 OCR，默认 **`true`**。
- `use_frame_detect`：是否先检测 A4 黑框，默认 **`true`**。
- `frame_aspect_tol`：A4 宽高比容差（相对 √2），默认 `0.22`。
- `frame_crop_margin_ratio`：框内缩，默认 `0.04`。
- `tesseract_psm`：Tesseract 页面模式，默认 **`7`**。
- `use_keyword_inference`：是否用赛规关键词推断标准播报句，默认 **`true`**。
- `adaptive_block_size`：自适应二值化邻域大小，默认 `51`。
- `adaptive_c`：自适应二值化偏移量，默认 `9`。

示例：

```bash
roslaunch text_recognition ocr_service.launch dark_threshold:=150 gray_on_frame_detect:=true
```
