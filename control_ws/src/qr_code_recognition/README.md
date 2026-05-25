# qr_code_recognition

识别板一二维码识别节点，按 `move_nav/Board1Decode` service 接口提供结果。

## 规则

识别板一是一张完整图片，最多包含四个二维码位置：

- 左上：`delivery_slot=1`
- 右上：`delivery_slot=2`
- 左下：`delivery_slot=3`
- 右下：`delivery_slot=4`

二维码内容只包含样本窗口字母，例如 `A`、`AB`、`ABC`，不包含
`blood`、`fluid` 等额外文本。节点根据二维码中心点所在象限推断
`delivery_slot`。

如果一张图中识别到多个二维码，节点只返回一个结果：优先选择样本数最少
的二维码；如果样本数相同，选择 `delivery_slot` 更小的二维码。

## 安装依赖

apt-get update
apt-get install libzbar0 -y
pip3 install pyzbar

## 运行

```bash
cd /CRAIC_SmartPharmacy/control_ws
catkin_make
source devel/setup.bash
rospack profile
roslaunch qr_code_recognition qr.launch
```

默认服务名：`/yaofang_vision/board1_decode`。

可选参数：

- `image_ready_timeout_sec`：读取图片前最多等待文件写入稳定的时间，默认 `0.2` 秒。
- `image_ready_poll_sec`：检查文件大小的轮询间隔，默认 `0.01` 秒。
