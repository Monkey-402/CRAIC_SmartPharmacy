# CRAIC 快速上手

药房赛项：**仿真调参 → 实车联调 → 控制任务**。细节见各子目录 README / QUICKSTART。

## 仓库结构

| 目录 | 用途 |
|------|------|
| `nav_sim_ws` | Gazebo 仿真 + 导航调参 |
| `nav_real_ws` | 实机 AMCL / Hector + move_base（不启 Gazebo） |
| `control_ws` | 药房主控、视觉、裁判 TCP |
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
| [`nav_real_ws/NAV_REAL_WS.md`](nav_real_ws/NAV_REAL_WS.md) | 实机与仿真差异 |
| [`judgement.md`](../judgement.md) | 裁判 JSON 规则 |
| [`lh.txt`](lh.txt) | 任务点手动 Nav Goal 命令 |
