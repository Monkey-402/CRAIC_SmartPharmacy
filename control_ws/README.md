# control_ws 说明（智慧社区示例工程）

这个工作空间主要用于智慧社区比赛任务，包含导航控制与任务分发模块。  
当前约定如下：

- `src/move_nav/src/control_node.cpp`：当前重构后的**抽象控制节点**（推荐作为二次开发入口）。
- 除上面这个文件外，`control_ws` 里的其余内容基本都可视为**智慧社区比赛方案示例**，可参考其功能与组织方式。

---

## 目录概览

- `src/move_nav/`
  - 导航控制包
  - `launch/control.launch`：控制节点启动入口
  - `launch/judgement_tcp_sender.launch`：裁判软件 TCP 上报
  - `launch/car_tcp_bridge.launch`：双车 TCP 通信桥接
  - `scripts/judgement_tcp_sender.py`：订阅 `JudgementReport`，经 TCP 发送 JSON 至裁判软件
  - `scripts/car_tcp_bridge.py`：ROS `Int32` 与对端整数双向 TCP 桥接
  - `msg/JudgementReport.msg`：裁判上报 ROS 消息定义
  - `src/control_node.cpp`：当前抽象版主控（视觉能力通过话题接口接入）
- `world/`
  - 比赛相关地图/世界文件资源

---

## 架构说明（当前推荐）

`move_nav` 中的 `control_node.cpp` 目前已改为“导航编排 + 任务分发”模式：

1. 到达指定导航点后抓图保存
2. 发布抽象任务请求（而不是直接调用某个视觉服务）
3. 订阅任务结果回传

默认话题：

- 请求：`smartcommunity/task_request`（`std_msgs/String`）
- 回传：`smartcommunity/task_result`（`std_msgs/String`）

请求消息内容示例（字符串键值对）：

`task_type=people_detection;goal_index=2;image_path=/home/xxx/snapshots/0.jpg;timestamp=1712345678.12`

你可以把任意视觉算法节点接到这个接口上，只要订阅请求并发布结果即可。

---

## 快速使用

### 1) 编译

在 `control_ws` 工作空间根目录执行：

```bash
catkin_make
source devel/setup.bash
```

### 2) 启动控制节点

```bash
roslaunch move_nav control.launch
```

### Melodic 视觉依赖（Python 2）

`control.launch` 中的二维码 / OCR 节点面向 **ROS Melodic（Python 2.7）**，请勿使用 `apt install python3-rospkg`（会与系统 `python-rospkg` 冲突）。

小车或开发机执行一次：

```bash
sudo apt-get update
sudo apt-get install -y \
  tesseract-ocr tesseract-ocr-chi-sim \
  libzbar0 python-opencv python-pip
sudo pip2 install 'pytesseract==0.2.9' 'pyzbar==0.1.8'
```

### 测试图像（模拟摄像头，与 control 分开启动）

默认主控订阅官方 **`/camera/rgb/image_raw`**（与实车 `uvc_camera`、仿真 Gazebo 一致）。离线测识别时开**两个终端**：

**终端 1**（主控 + 视觉，常驻）：

```bash
source devel/setup.bash
roslaunch move_nav control.launch \
  image_topic:=/yaofang_test/image_raw \
  mock_navigation:=true max_rounds:=1
```

**终端 2**（测试图发布，可随时 Ctrl+C 重启换图）：

```bash
source devel/setup.bash
roslaunch move_nav test_image_publisher.launch
# 换图示例：
# roslaunch move_nav test_image_publisher.launch image_path:=/path/to/other.png
```

实车（只用真相机，不启终端 2）：

```bash
roslaunch move_nav control.launch
```

---

## 裁判软件 TCP 上报（judgement_tcp_sender）

用于 CRAIC 智慧药房赛项：订阅 ROS 消息，按规则以 **1–2 Hz** 通过 **TCP/IP** 向裁判软件发送 JSON。  
规则详见仓库根目录 `judgement.md`。

### 消息定义

话题默认：`/judgement/report`（`move_nav/JudgementReport`）

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | string | 小车编号，`"1"` 或 `"2"` |
| `speed` | float64 | 速度（m/s） |
| `odom` | float64[] | 地图坐标 `[x, y]`（m） |
| `task` | string | 当前任务，如 `"A"`、`"1"`、`"R"` |
| `CV1` | string | 识别板二结果，如 `"WAIT-8"` |
| `CV2` | string | 二维码结果，如 `"AB-1"` |

发送 JSON 示例：

```json
{"id":"1","speed":0.2,"odom":[2.2,1.0],"task":"A","CV1":"WAIT-8","CV2":"AB-1"}
```

### 启动

赛前将 `server_ip`、`server_port` 改为现场公布的裁判软件地址：

```bash
roslaunch move_nav judgement_tcp_sender.launch \
  server_ip:=192.168.1.102 \
  server_port:=8888
```

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `server_ip` | `192.168.1.100` | 裁判软件 IP |
| `server_port` | `8888` | 裁判软件端口 |
| `send_rate` | `1.5` | 发送频率（Hz），规则要求 1–2 |
| `input_topic` | `/judgement/report` | 订阅话题 |

### 测试发布

```bash
rostopic pub /judgement/report move_nav/JudgementReport \
  "id: '1'
speed: 0.2
odom: [2.2, 1.0]
task: 'A'
CV1: 'WAIT-8'
CV2: 'AB-1'"
```

### 注意

- 小车与裁判软件需在同一局域网；防火墙需放行对应 TCP 端口。
- 节点缓存最新一条消息并按固定频率发送；断线会自动重连。
- 其他节点只需持续发布 `JudgementReport`，无需自行处理 TCP。

---

## 双车协调（home / standby + CarLink）

识别过的二维码会从屏幕消失，**无需**双车同步占用表。协调仅负责 **起点轮流**；板一 `delivery_slot` 优先级（1→3→4 优先，2 最低）为**各车本地**逻辑。

| 小车 | IP | 初始站位 | TCP |
|------|-----|----------|-----|
| 1 号车 | `192.168.124.3` | `home` | server :9000 |
| 2 号车 | `192.168.124.9` | `standby` | client → 1 号车 |

### 一键启动（参数均在 `config/*.yaml`，launch 无业务参数）

| 场景 | 1 号车 | 2 号车 | 配置文件 |
|------|--------|--------|----------|
| 双机仿真 104/105 | `sim_car1.launch` | `sim_car2.launch` | `config/sim_car1.yaml`, `sim_car2.yaml` |
| 赛场实车 124.3/124.9 | `real_car1.launch` | `real_car2.launch` | `config/real_car1.yaml`, `real_car2.yaml` |

```bash
# 仿真：104 先起，105 后起
roslaunch move_nav sim_car1.launch
roslaunch move_nav sim_car2.launch

# 实车（含裁判 TCP）
roslaunch move_nav real_car1.launch
roslaunch move_nav real_car2.launch
```

改 standby、peer_ip、裁判地址等请只编辑对应 **yaml**，不要改 launch。

双车模式下 **TCP 未连接前不会导航**；2 号车 **必须收到 1 号 `ROUND_DONE`** 且在 **home** 才会开工。

### CarLink 话题与 TCP JSON

| 话题 | 类型 |
|------|------|
| `/car_link/send` | `move_nav/CarLink` |
| `/car_link/recv` | `move_nav/CarLink` |

线格式（一行一条 JSON）：`{"v":1,"type":1,"from_id":"1","seq":10,"station":3,"delivery_slot":3}`

| type | 含义 |
|------|------|
| 0 `HEARTBEAT` | 周期站位 |
| 1 `SCAN_OK` | 板一已接单，对端 standby→home |
| 2 `ROUND_DONE` | 本轮结束在 standby，对端 home 可开工 |

### 预备点

`standby_x/y/yaw` 在 `config/sim_car*.yaml` / `config/real_car*.yaml` 中配置。

### 板一 slot 优先级（单车同样生效）

| 参数 | 默认 |
|------|------|
| `deprioritize_delivery_slot` | `2` |
| `slot2_max_visits_before_accept` | `2` |

扫到 slot 2 时在板一重扫，visit 达上限后才接受 slot 2。

---

## 二次开发建议

- 以 `control_node.cpp` 为主，视觉能力通过独立节点接入 `smartcommunity/task_request` / `task_result`。
- 建议后续把 `task_request/task_result` 从 `std_msgs/String` 升级为自定义消息（字段更清晰、可扩展）。

---

## 任务点（`control_node_yaofang_service_template.cpp`）

`GOAL_LIST` 每项格式：`{x, y, yaw, "name"}`。终点航向容差由导航栈 TEB yaml（`yaw_goal_tolerance`）统一配置。

---

## 备注

在正式比赛或部署前，建议统一检查：

- 抓图保存目录与权限：默认 **`control_ws/src/snapshots/`**（QR 裁剪 `*_slot1..4.jpg`、OCR `*_ocr_roi.jpg` / `*_ocr_bin.jpg` 同目录）
- 摄像头话题名：默认 **`/camera/rgb/image_raw`**（官方）；离线测试图 **`/yaofang_test/image_raw`**
- 实车 IP：1 号车 **`192.168.124.3`**，2 号车 **`192.168.124.9`**
- 地图/world 与导航参数匹配
