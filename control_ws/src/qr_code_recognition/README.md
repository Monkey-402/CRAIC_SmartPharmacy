# qr_code_recognition

识别板一二维码识别节点，按 `move_nav/Board1Decode` service 接口提供结果。

## 规则

识别板一是一张完整图片，**最多**四个二维码；**不要求四格都扫到**，全图 pyzbar 扫到 **≥1 个** 且内容含 A/B/C、slot 在 1–4 即判定成功。

- 左上：`delivery_slot=1`
- 右上：`delivery_slot=2`
- 左下：`delivery_slot=3`
- 右下：`delivery_slot=4`

## 处理流程

1. 对灰度图取亮白像素（`bright_thresh`）求坐标均值，作为参考中心（非图像几何中心）
2. 全图 pyzbar 扫码（1.0× / 1.5× / 2.0×，失败则试反色图）
3. 按码中心相对参考中心的象限分配 slot 1–4
4. 多格有码时选一个：默认 **样本数最多**；`prefer_fewest_samples=true` 时选 **样本数最少**（双车累计第 4 轮）

## 裁剪图保存

与主控 snapshot 同目录，按检测到的 QR 外接框裁剪（非四黑框分格）：

- `snapshots/0.jpg`
- `snapshots/0_slot1.jpg` …（仅有码的 slot 会写出）

## 参数（`qr.launch` / `control.launch` 的 `qr_*`）

| 参数 | 默认 | 说明 |
|------|------|------|
| `bright_thresh` | 165 | 亮白像素灰度下限（略放宽，更易检出浅白区） |
| `min_bright_pixels` | 30 | 亮像素过少则退回图像中心 |
| `min_qr_area` | 120 | pyzbar 框最小面积（px²） |
| `crop_margin_ratio` | 0.15 | 保存 slot 裁剪图时的边距比例 |

## 安装依赖

```bash
apt-get update
apt-get install libzbar0 python-opencv python-pip -y
pip2 install 'pyzbar==0.1.8'
```

## 离线测试

```bash
python /path/to/snapshots/decode_board1.py image.jpg --save-debug
```

## 启动

```bash
roslaunch qr_code_recognition qr.launch
# 或经 move_nav control.launch / real_car1.launch 一并拉起
```
