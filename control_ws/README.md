# control_ws

智慧药房赛项 **任务主控与视觉** 工作空间：导航点编排、识别板一/二、语音播报、裁判 TCP 上报、双车协调。

**启动命令** → [`QUICKSTART.md`](QUICKSTART.md)  
**实车全栈** → [`../QUICKSTART.md`](../QUICKSTART.md)

## 架构

```
move_nav（主控）
  ├─ 订阅 /camera/rgb/image_raw → 截图
  ├─ 调用 Board1Decode / Board2Decode 服务
  ├─ move_base 逐点导航（GOAL_LIST）
  ├─ 发布 /judgement/report（任务中）
  └─ 双车模式：CarLink + car_tcp_bridge

qr_code_recognition     → 板一二维码（pyzbar）
text_recognition        → 板二 OCR（Tesseract，默认）
board2_paddle_ocr       → 板二 OCR（Paddle HTTP，可选）
```

主控入口：`src/move_nav/src/control_node_yaofang_service_template.cpp`（服务版，实赛用）；`control_node_yaofang_template.cpp` 为话题版参考实现。

## 包说明

| 包 | README | 职责 |
|----|--------|------|
| `move_nav` | 本页 | 主控、launch、音频、裁判/双车 TCP |
| `qr_code_recognition` | [README](src/qr_code_recognition/README.md) | 板一全图扫码、slot 分配、样本数优选 |
| `text_recognition` | [README](src/text_recognition/README.md) | 板二 Tesseract + A4 黑框检测 |
| `board2_paddle_ocr` | [README](src/board2_paddle_ocr/README.md) | 板二 Paddle HTTP 后端 |

## 视觉依赖（Melodic / Python 2）

`control.launch` 内 QR / Tesseract 节点为 **Python 2.7**。勿 `apt install python3-rospkg`（与 `python-rospkg` 冲突）。

```bash
sudo apt-get update
sudo apt-get install -y \
  tesseract-ocr tesseract-ocr-chi-sim \
  libzbar0 python-opencv python-pip
sudo pip2 install 'pytesseract==0.2.9' 'pyzbar==0.1.8'
```

板二改用 Paddle：见 [`board2_paddle_ocr/README.md`](src/board2_paddle_ocr/README.md)（Python 3 conda 服务 + `use_paddle_ocr:=true`）。

## 裁判软件 TCP 上报

节点 `judgement_tcp_sender` 订阅 `/judgement/report`（`move_nav/JudgementReport`），以 **1–2 Hz** 向裁判软件发 JSON。规则见仓库根 [`judgement.md`](../judgement.md)。

主控在 **home / standby** 不上报；**任务中**（`STATION_ON_MISSION`）按 `judgement_report_rate`（默认 1.5 Hz）发布。

| 字段 | 说明 |
|------|------|
| `id` | 车号 `"1"` / `"2"` |
| `speed` | 速度 m/s |
| `odom` | 地图坐标 `[x, y]` |
| `task` | 当前阶段 `A`/`B`/`C`/`1`–`4`/`R` |
| `CV1` | 板二，如 `WAIT-8` |
| `CV2` | 板一，如 `AB-1` |

单独测 TCP、launch 参数 → [`QUICKSTART.md`](QUICKSTART.md) §裁判。

## 双车协调（home / standby + CarLink）

识别过的二维码会从屏幕消失，**无需**双车同步占用表。

| 规则 | 说明 |
|------|------|
| 2 号首次出工 | standby 等到对端 **REACHED_ABC** 后再去 home |
| `ROUND_DONE` | 本车 **到达 standby** 后发送 |
| home 出工 | 收到对端 `ROUND_DONE` 后再开始下一轮（1 号首轮除外） |
| 板一扫码 | 在 `board1_scan`；多码默认样本数最多，双车累计第 4 轮优先 2 个样本 |

| 小车 | IP | 初始站位 | TCP |
|------|-----|----------|-----|
| 1 号 | `192.168.124.3` | home | server :9000 |
| 2 号 | `192.168.124.9` | standby | client → 1 号 |

业务参数在 `config/sim_car*.yaml` / `real_car*.yaml`；**勿**对主控节点使用 `clear_params="true"`。

开赛：到 home/standby 后等车际 + 裁判 TCP，**1 号** 5 秒倒计时并广播 `MATCH_START`。参数 `enable_prestart_countdown`、`prestart_countdown_sec`。

CarLink 类型：`HEARTBEAT(0)`、`ROUND_DONE(2)`、`REACHED_ABC(5)`、`MATCH_START(6)` 等。话题 `/car_link/send`、`/car_link/recv`。

一键 launch 与 yaml 说明 → [`QUICKSTART.md`](QUICKSTART.md) §双车。

## 任务点

`control_node_yaofang_service_template.cpp` 中 `GOAL_LIST`：`{x, y, yaw, "name"}`。航向容差由导航 TEB yaml 的 `yaw_goal_tolerance` 统一配置。手动 Nav Goal 坐标见 [`lh.txt`](../lh.txt)。

## 快照与话题

| 项 | 默认 |
|----|------|
| 相机 | `/camera/rgb/image_raw` |
| 离线测图 | `/yaofang_test/image_raw` + `test_image_publisher.launch` |
| 截图目录 | `control_ws/snapshots/`（QR/OCR 调试图同目录） |

## 参数优先级

| 入口 | 业务参数 | 调试参数 |
|------|----------|----------|
| `sim_car*` / `real_car*` | profile **yaml** | launch 可透传 `mock_navigation` 等 |
| `control.launch` | yaml + `dual_car_mode` 默认 false | launch node param 覆盖 yaml |
