# CRAIC

智慧药房赛项 ROS 工作空间：**仿真导航** → **实机导航** → **任务控制**。

**操作步骤 → [`QUICKSTART.md`](QUICKSTART.md)**（只保留命令；说明见下文与各子目录文档。）

## 工作空间

| 目录 | 说明 |
|------|------|
| [`nav_sim_ws`](nav_sim_ws/) | Gazebo 仿真，`AMCL + move_base + TEB` |
| [`nav_real_ws`](nav_real_ws/) | 实机导航（默认不启 Gazebo） |
| [`control_ws`](control_ws/) | 药房主控、二维码 / 板二 OCR、裁判 TCP；板二可选 [`board2_paddle_ocr`](control_ws/src/board2_paddle_ocr/) |
| [`robot_ws`](robot_ws/) | 实车底盘 launch 模板（拷到小车 `~/robot_ws`） |

**实车 IP**：1 号 `192.168.124.3`，2 号 `192.168.124.9`。

## 推荐流程

1. `nav_sim_ws` 调通导航与参数  
2. `nav_real_ws` + 实车底盘联调  
3. `control_ws` 跑完整药房任务  

## 实车前提

- 小车已装官方 `eprobot_start`、`lslidar_driver` 等。  
- 本仓库 `robot_ws/src/eprobot_chassis_bringup` 已拷到小车 `~/robot_ws/src` 并 `catkin_make`。  
- 终端 A 默认 `pub_odom_tf:=false`（EKF 发 odom TF）；**不用 EKF** 时 A 改为 `pub_odom_tf:=true`，导航用 `nav_real_amcl_no_ekf*.launch`。

## 视觉依赖（Melodic / Python 2）

`control.launch` 中 QR / Tesseract OCR 为 **Python 2.7**。勿 `apt install python3-rospkg`（与 `python-rospkg` 冲突）。

```bash
sudo apt-get update
sudo apt-get install -y \
  tesseract-ocr tesseract-ocr-chi-sim \
  libzbar0 python-opencv python-pip
sudo pip2 install 'pytesseract==0.2.9' 'pyzbar==0.1.8'
```

## 板二 OCR：Tesseract 与 Paddle

| 方式 | 说明 |
|------|------|
| **默认** | `control.launch` 内 Tesseract（`text_recognition`），依赖见上一节 |
| **Paddle** | 独立 HTTP 服务（Python 3）+ `use_paddle_ocr:=true`；中文屏显更稳 |

Paddle 一次性安装（需 `curl`；树莓派可用 `CONDA_MIRROR=tsinghua ./setup_paddle_conda.sh`）：

```bash
source ~/craic/control_ws/devel/setup.bash
roscd board2_paddle_ocr
chmod +x setup_paddle_conda.sh run_paddle_ocr_server.sh
./setup_paddle_conda.sh
```

`control.launch` 参数：`use_paddle_ocr`（默认 `false`）、`paddle_ocr_url`（默认 `http://127.0.0.1:8765`）、`paddle_ocr_timeout`（默认 `120`）。Docker 跑 Melodic 时需 **`--net=host`**，容器才能访问宿主机 `8765`。

详见 [`control_ws/src/board2_paddle_ocr/README.md`](control_ws/src/board2_paddle_ocr/README.md)。

## TEB 参数预设

文件目录：`nav_real_ws/src/car_sim/param/`（仿真：`nav_sim_ws/.../param/` 同名）。

| 预设文件 | 用途 | `max_vel_x` |
|----------|------|-------------|
| `base_local_planner_params_TEB.yaml` | 默认保守 | 1.0 |
| `base_local_planner_params_TEB_smooth.yaml` | 顺滑路径（速度同默认，减轻弯口反复修角） | 1.0 |
| `base_local_planner_params_TEB_conservative_half.yaml` | 一半速度 | 0.5 |
| `base_local_planner_params_TEB_official_max_vel.yaml` | 对齐 `robot_ws_official` 最大速度 | 1.2 |

导航 launch 通过 **`teb_config:=$(rospack find car_sim)/param/<文件名>`** 切换；**须重启导航 launch** 才从 yaml 加载。`rosparam set` 仅临时生效。

验证示例：

```bash
rosparam get /move_base/TebLocalPlannerROS/max_vel_x
rosparam get /move_base/TebLocalPlannerROS/global_plan_viapoint_sep   # smooth 应为 0.8
rosparam get /move_base/TebLocalPlannerROS/enable_homotopy_class_planning  # 须 false
```

完整 launch 列表、临时改参、性能基准：**[`nav_real_ws/QUICKSTART.md`](nav_real_ws/QUICKSTART.md) §6–§7**。

## 其它常改参数

- Costmap：`nav_*_ws/src/car_sim/param/costmap_common_params.yaml`  
- 任务点：`control_ws/src/move_nav/src/control_node_yaofang_service_template.cpp` 中 `GOAL_LIST`  
- 双车协调、开赛 5 秒倒计时：`control_ws/README.md` 双车一节；参数在 `real_car1.yaml` / `sim_car1.yaml`

## 常用话题

| 话题 | 说明 |
|------|------|
| `/scan_filtered` | 导航激光 |
| `/odom` | 里程计（EKF 模式多为 `/odometry/filtered`） |
| `/camera/rgb/image_raw` | 主控 / 视觉 |
| `/move_base` | 导航 action |

## 仿真说明

- 推荐 `nav_sim_amcl.launch`，话题与实车对齐。  
- 仿真相机：`car_simple.urdf` 为 640×480@30Hz，话题 `/camera/rgb/image_raw`。  
- 建图给实车：`hector_sim.launch` → `map_save.launch` → 拷贝 `map_sim.*` 到 `nav_real_ws/src/car_sim/map/`。  
- 宿主机无 Melodic：见 [`nav_sim_ws/QUICKSTART.md`](nav_sim_ws/QUICKSTART.md) §7（Docker）。

## 文档索引

| 文档 | 用途 |
|------|------|
| **[QUICKSTART.md](QUICKSTART.md)** | 总操作命令（首选） |
| [nav_sim_ws/QUICKSTART.md](nav_sim_ws/QUICKSTART.md) | 仿真、Docker |
| [nav_real_ws/QUICKSTART.md](nav_real_ws/QUICKSTART.md) | 实车 RViz、TEB §6、验收排错 |
| [control_ws/README.md](control_ws/README.md) | 主控、裁判 TCP、双车 |
| [nav_real_ws/NAV_REAL_WS.md](nav_real_ws/NAV_REAL_WS.md) | 实机与仿真差异 |
| [judgement.md](../judgement.md) | 裁判 JSON |
| [lh.txt](lh.txt) | 任务点手动 Nav Goal |
| [GIT_WORKFLOW.md](GIT_WORKFLOW.md) | Git 规范 |
