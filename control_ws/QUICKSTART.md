# control_ws 快速启动

> 架构、裁判 TCP、双车原理见 [README.md](README.md)。实车 A～D 终端总览见 [../QUICKSTART.md](../QUICKSTART.md)。

## 1) 编译

```bash
cd ~/craic/control_ws
catkin_make
source devel/setup.bash
```

## 2) 实车双车（赛场）

**1 号车**（`192.168.124.3`，与 `nav_real_amcl_car1` 同机）：

```bash
source ~/craic/control_ws/devel/setup.bash
roslaunch move_nav real_car1.launch use_paddle_ocr:=true
```

**2 号车**（`192.168.124.9`，与 `nav_real_amcl_car2` 同机）：

```bash
roslaunch move_nav real_car2.launch use_paddle_ocr:=true
```

改 standby、裁判 IP、双车 TCP 等：只编辑 `config/real_car1.yaml` / `real_car2.yaml`。

## 3) 仿真双车

导航另开终端（见 [`nav_sim_ws/QUICKSTART.md`](../nav_sim_ws/QUICKSTART.md)）后：

```bash
source ~/craic/control_ws/devel/setup.bash
roslaunch move_nav sim_car1.launch    # 104 先起
roslaunch move_nav sim_car2.launch    # 105 后起
```

## 4) 单车 / 调试

```bash
# 实车真相机
roslaunch move_nav control.launch

# 仿真（放宽 QR 黑框）
roslaunch move_nav control_sim.launch

# 单轮 + 模拟导航（不连 move_base）
roslaunch move_nav yaofang_service_mock.launch max_rounds:=1 mock_navigation:=true
```

## 5) 离线测视觉（mock 相机）

**终端 1** — 主控 + 视觉：

```bash
source devel/setup.bash
roslaunch move_nav control.launch \
  image_topic:=/yaofang_test/image_raw \
  mock_navigation:=true max_rounds:=1
```

**终端 2** — 静态测试图：

```bash
roslaunch move_nav test_image_publisher.launch
# roslaunch move_nav test_image_publisher.launch image_path:=/path/to/other.png
```

## 6) Paddle OCR（可选）

**终端 D** — HTTP 服务（宿主机或 `--net=host` 容器内）：

```bash
source ~/craic/control_ws/devel/setup.bash
roscd board2_paddle_ocr && ./run_paddle_ocr_server.sh
```

主控启用：`use_paddle_ocr:=true`（见 §2）。首次安装见 [`board2_paddle_ocr/README.md`](src/board2_paddle_ocr/README.md)。

## 7) 裁判 TCP 单独测

```bash
roslaunch move_nav judgement_tcp_sender.launch \
  server_ip:=192.168.1.102 server_port:=8888

rostopic pub /judgement/report move_nav/JudgementReport \
  "id: '1'
speed: 0.2
odom: [2.2, 1.0]
task: 'A'
CV1: 'WAIT-8'
CV2: 'AB-1'"
```

## 8) 视觉节点单独起

```bash
roslaunch qr_code_recognition qr.launch
roslaunch text_recognition ocr_service.launch
```

通常由 `control.launch` / `real_car*.launch` 一并拉起。
