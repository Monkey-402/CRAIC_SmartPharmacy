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

## 双车 TCP 通信（car_tcp_bridge）

用于两车协同：在 ROS 内订阅/发布整数，车与车之间经 **TCP** 传输，不依赖跨车 ROS 通信。

### 话题

| 方向 | 默认话题 | 类型 | 说明 |
|------|----------|------|------|
| 发出 | `/car_link/send` | `std_msgs/Int32` | 本车要发给对端的数字 |
| 接收 | `/car_link/recv` | `std_msgs/Int32` | 对端发来的数字 |

TCP 协议：一行一个整数，如 `42\n`。连接建立后**双向**收发。

### 角色

| 角色 | 行为 | 建议 |
|------|------|------|
| `server` | 监听端口，等对端连接 | 1 号车 |
| `client` | 主动连接对端 IP | 2 号车 |

### 启动

**1 号车（server）：**

```bash
roslaunch move_nav car_tcp_bridge_car1.launch
# 或
roslaunch move_nav car_tcp_bridge.launch role:=server port:=9000
```

**2 号车（client，`peer_ip` 填 1 号车 IP）：**

```bash
roslaunch move_nav car_tcp_bridge_car2.launch
# 或
roslaunch move_nav car_tcp_bridge.launch role:=client peer_ip:=192.168.1.101 port:=9000
```

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `role` | `client` | `server` 或 `client` |
| `peer_ip` | `192.168.1.102` | client 模式下对端 IP |
| `port` | `9000` | TCP 端口 |
| `send_topic` | `/car_link/send` | 发送订阅话题 |
| `recv_topic` | `/car_link/recv` | 接收发布话题 |

### 测试

**A 车发送：**

```bash
rostopic pub /car_link/send std_msgs/Int32 "data: 1"
```

**B 车接收：**

```bash
rostopic echo /car_link/recv
```

反向同理：B 发 `/car_link/send`，A 收 `/car_link/recv`。

### 注意

- 两台设备需在同一网段；**server 端**需放行 TCP 端口（默认 9000）。
- Windows 默认可能拦截 ping，但不影响 TCP；以 `nc -zv <对端IP> 9000` 验证连通性更可靠。
- 断线后 client 会自动重连；server 断开后会重新等待连接。

---

## 二次开发建议

- 以 `control_node.cpp` 为主，视觉能力通过独立节点接入 `smartcommunity/task_request` / `task_result`。
- 建议后续把 `task_request/task_result` 从 `std_msgs/String` 升级为自定义消息（字段更清晰、可扩展）。

---

## 备注

在正式比赛或部署前，建议统一检查：

- 抓图保存目录与权限（如 `/root/snapshots`）
- 摄像头话题名（如 `/camera/image_raw`）
- 地图/world 与导航参数匹配
