# qr_code_recognition

识别板一二维码识别节点，按 `move_nav/Board1Decode` service 接口提供结果。

## 规则

识别板一是一张完整图片，最多包含四个带黑框的正方形窗口：

- 左上：`delivery_slot=1`
- 右上：`delivery_slot=2`
- 左下：`delivery_slot=3`
- 右下：`delivery_slot=4`

处理流程：

1. 对整图做反色二值化 + 形态学闭运算，检测四个黑框轮廓（近似正方形）
2. 按框中心位置排序为 slot 1–4，向内缩进裁剪
3. 在每个裁剪图内单独扫码；空框会保存裁剪图但无 QR 结果
4. 多格有码时按比赛规则选一个：样本数最少，相同则 slot 更小

二维码内容只包含样本窗口字母，例如 `A`、`AB`、`ABC`。

## 裁剪图保存

与主控 snapshot 同目录，例如：

- `snapshots/0.jpg`
- `snapshots/0_slot1.jpg` … `snapshots/0_slot4.jpg`

## 安装依赖

```bash
apt-get update
apt-get install libzbar0 python-opencv python-pip -y
pip2 install 'pyzbar==0.1.8'
```

## 运行

```bash
cd control_ws
catkin_make
source devel/setup.bash
roslaunch qr_code_recognition qr.launch
```

默认服务名：`/yaofang_vision/board1_decode`。
