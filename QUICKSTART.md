# CRAIC 快速上手

药房赛项：**仿真调参 → 实车联调 → 控制任务**。细节见各子目录 README / QUICKSTART。

## 仓库结构

| 目录 | 用途 |
|------|------|
| `nav_sim_ws` | Gazebo 仿真 + 导航调参 |
| `nav_real_ws` | 实机 AMCL / Hector + move_base（不启 Gazebo） |
| `control_ws` | 药房主控、视觉（QR + 板二 OCR）、裁判 TCP；板二可选 `board2_paddle_ocr` |
| `robot_ws` | 实车底盘 launch 模板（复制到小车上 `~/robot_ws`） |

**实车 IP**：1 号车 `192.168.124.3`，2 号车 `192.168.124.9`。

## 一次性编译

```bash
cd ~/craic/nav_sim_ws && catkin_make    # 仿真
cd ~/craic/nav_real_ws && catkin_make   # 实机导航
cd ~/craic/control_ws && catkin_make    # 主控 + 视觉
```

新终端使用前：`source <工作空间>/devel/setup.bash`。

---

## 实车（推荐流程）

**前提**：小车已安装官方 `eprobot_start`、`lslidar_driver` 等；本仓库 `robot_ws/src/eprobot_chassis_bringup` 已拷到小车 `~/robot_ws/src` 并编译。

### 1. 同步代码（开发机 → 小车）

```bash
cd ~/craic
./sync_to_robot.sh                         # 1 号车
./sync_to_robot.sh EPRobot@192.168.124.9  # 2 号车
```

### 2. 小车上启动（同一 ROS master）

**终端 A — 底盘 + 雷达 + 相机**

```bash
export ROBOT_TYPE=EPRobotV2.3
source ~/robot_ws/devel/setup.bash
roslaunch eprobot_chassis_bringup chassis.launch
# 默认 pub_odom_tf:=false，供 EKF 单独发 odom TF
```

**终端 B — 导航**

```bash
source ~/craic/nav_real_ws/devel/setup.bash
# EKF + AMCL（默认，推荐）
roslaunch car_sim nav_real_amcl.launch no_rviz:=true

# 不用 EKF：终端 A 改为 pub_odom_tf:=true，再：
# roslaunch car_sim nav_real_amcl_no_ekf.launch no_rviz:=true

# 无静态地图、需在线建图：nav_real_hector.launch（底盘 pub_odom_tf:=true）
```

RViz 在本机开：见下文「笔记本连小车」。

**终端 C — 药房主控（可选）**

```bash
source ~/craic/control_ws/devel/setup.bash
roslaunch move_nav control.launch
# 单轮测试：yaofang_service_mock.launch max_rounds:=1
```

默认识别板二走 **Tesseract**（`text_recognition`），与 QR 一起在 `control.launch` 里拉起，无需另开终端。若要改用 **PaddleOCR**，见下节。

**Melodic 视觉依赖（Python 2，小车或开发机执行一次）**

`control.launch` 中的二维码 / OCR 节点为 **Python 2.7**，请勿 `apt install python3-rospkg`（会与系统 `python-rospkg` 冲突）。

```bash
sudo apt-get update
sudo apt-get install -y \
  tesseract-ocr tesseract-ocr-chi-sim \
  libzbar0 python-opencv python-pip
sudo pip2 install 'pytesseract==0.2.9' 'pyzbar==0.1.8'
```

### 识别板二 OCR（Paddle，可选）

| 方式 | 启动 | 说明 |
|------|------|------|
| **默认** | 仅 `control.launch` | Tesseract（依赖见上一段 apt / pip2） |
| **Paddle** | HTTP 服务 + `use_paddle_ocr:=true` | 中文屏显更稳；Py3 服务与 Py2 ROS 节点分离 |

**一次性安装**（开发机或小车，仅需一次；需 `curl`，Docker 内可先 `apt-get install -y curl`）：

```bash
source ~/craic/control_ws/devel/setup.bash
roscd board2_paddle_ocr
chmod +x setup_paddle_conda.sh run_paddle_ocr_server.sh
./setup_paddle_conda.sh
```

**实车 / 联调：在终端 C 之前或并行开「终端 D — Paddle HTTP」**

```bash
source ~/craic/control_ws/devel/setup.bash
roscd board2_paddle_ocr
./run_paddle_ocr_server.sh
# 健康检查（另开终端）：
# rosrun board2_paddle_ocr paddle_ocr_client.py --health
```

**终端 C 启用 Paddle 桥接**（仍暴露同一 `/yaofang_vision/board2_decode`，主控无需改代码）：

```bash
source ~/craic/control_ws/devel/setup.bash
roslaunch move_nav control.launch use_paddle_ocr:=true
# 服务不在本机时：paddle_ocr_url:=http://<IP>:8765
```

`control.launch` 相关参数：`use_paddle_ocr`（默认 `false`）、`paddle_ocr_url`（默认 `http://127.0.0.1:8765`）、`paddle_ocr_timeout`（默认 `120`）。Docker 跑 Melodic 时请 **`--net=host`**，容器内才能访问宿主机上的 `8765` 端口。

**离线测图**（不启整车，只验板二；需先起 Paddle HTTP）：

```bash
# 仓库外调试目录（若已挂载 ~/CAIR/snapshots）：
python /root/snapshots/decode_board2_paddle.py /root/snapshots/11.jpg

# 或 ROS 包内：
rosrun board2_paddle_ocr decode_board2_paddle.py /path/to/11.jpg \
  --snapshot-dir ~/CAIR/craic/control_ws/src/snapshots
```

细节见 [`control_ws/src/board2_paddle_ocr/README.md`](control_ws/src/board2_paddle_ocr/README.md)。

### 3. 笔记本连小车（master 在小车）

```bash
export ROS_MASTER_URI=http://192.168.124.3:11311   # 或 .9
export ROS_IP=<本机局域网 IP>
source ~/craic/nav_real_ws/devel/setup.bash
rviz -d ~/craic/nav_real_ws/src/car_sim/rviz/nav.rviz
```

AMCL：Fixed Frame `map` → **2D Pose Estimate** → **2D Nav Goal**。

### 4. 验收（导航中必看）

发 Nav Goal 后：

```bash
rostopic hz /cmd_vel          # 应 ~7.5–8 Hz；若 ~1 Hz 见 nav_real_ws/QUICKSTART §7
rostopic hz /scan_filtered
rosparam get /move_base/TebLocalPlannerROS/enable_homotopy_class_planning  # 须 false
```

---

## 仿真（Gazebo）

```bash
source ~/craic/nav_sim_ws/devel/setup.bash
roslaunch car_sim nav_sim_amcl.launch   # 推荐，话题与实车对齐
# 或：nav_sim.launch
```

**终端 C — 主控 + 视觉（仿真建议放宽 QR 参数）**

```bash
source ~/craic/control_ws/devel/setup.bash
roslaunch move_nav control_sim.launch
# 等价：roslaunch move_nav control.launch qr_sim_mode:=true
```

仿真相机已在 `car_simple.urdf` 固定为 **640×480@30Hz**，话题 `/camera/rgb/image_raw`（与实车一致）。自检：

```bash
rostopic echo /camera/rgb/image_raw/header -n1
rostopic hz /camera/rgb/image_raw
```

建图保存给实车：`hector_sim.launch` → `map_save.launch` → 拷贝 `map_sim.*` 到 `nav_real_ws/src/car_sim/map/`。

**Docker（宿主机无 Melodic）**：`docker build -t craic:melodic .` 后挂载 `~/craic` 进容器；详见 [`nav_sim_ws/QUICKSTART.md`](nav_sim_ws/QUICKSTART.md) §7。

---

## 常用话题

| 话题 | 说明 |
|------|------|
| `/scan_filtered` | 导航激光 |
| `/odom` | 里程计 |
| `/camera/rgb/image_raw` | 主控 / 视觉 |
| `/move_base` | 导航 action |

---

## 最常改的参数

- TEB：`nav_*_ws/src/car_sim/param/base_local_planner_params_TEB.yaml`
- Costmap：`nav_*_ws/src/car_sim/param/costmap_common_params.yaml`
- 任务点：`control_ws/src/move_nav/src/control_node_yaofang_service_template.cpp` 中 `GOAL_LIST`

---

## 延伸阅读

| 文档 | 内容 |
|------|------|
| [`nav_real_ws/QUICKSTART.md`](nav_real_ws/QUICKSTART.md) | 实车 RViz、性能基准、故障排查 |
| [`nav_sim_ws/QUICKSTART.md`](nav_sim_ws/QUICKSTART.md) | 仿真、Docker 完整步骤 |
| [`control_ws/README.md`](control_ws/README.md) | 视觉依赖、裁判 TCP、双车通信 |
| [`control_ws/src/board2_paddle_ocr/README.md`](control_ws/src/board2_paddle_ocr/README.md) | 板二 PaddleOCR HTTP 与 `use_paddle_ocr` |
| [`nav_real_ws/NAV_REAL_WS.md`](nav_real_ws/NAV_REAL_WS.md) | 实机与仿真差异 |
| [`judgement.md`](../judgement.md) | 裁判 JSON 规则 |
| [`lh.txt`](lh.txt) | 任务点手动 Nav Goal 命令 |
