# text_recognition

识别板二文字识别节点，按 `move_nav/Board2Decode` service 接口提供结果。

## 规则

识别板二是一张屏幕截图，节点读取图片路径后使用 Tesseract OCR 识别
中文和英文文本，并返回：

- `speech_text`：识别到的完整文字
- `wait_seconds`：从文字中提取出的等待秒数

等待秒数只匹配带时间单位的数字，例如 `30 秒`、`45s`、`60 seconds`。
这样可以避免把窗口号、取药口编号、日期或编号误判为等待时间。

图片会先进行 OCR 前处理：灰度化、高斯滤波、二值化。默认使用大津法
`otsu`，如果现场屏幕反光或光照不均明显，可以切换为自适应二值化
`adaptive`。

## 安装依赖

apt-get update
apt-get install tesseract-ocr tesseract-ocr-chi-sim -y
pip3 install pytesseract numpy opencv-python

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
- `threshold_method`：二值化方式，默认 `otsu`，可改为 `adaptive`。
- `adaptive_block_size`：自适应二值化邻域大小，仅 `adaptive` 生效，默认 `31`。
- `adaptive_c`：自适应二值化偏移量，仅 `adaptive` 生效，默认 `11`。

示例：

```bash
roslaunch text_recognition ocr_service.launch threshold_method:=adaptive adaptive_block_size:=51 adaptive_c:=9
```
