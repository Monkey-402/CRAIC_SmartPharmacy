# Quickstart（实机导航）

> **总览**：见 [`../QUICKSTART.md`](../QUICKSTART.md)。下文为实车 RViz、性能验收与排错补充。

实机默认**不启 Gazebo**，在宿主 ROS（常见为 Noetic/Melodic）上运行。

## 1) 一次性准备

```bash
cd ~/craic/nav_real_ws
catkin_make
# 若与 nav_sim_ws 联编，先 source nav_sim_ws 再编译本工作空间
```

## 2) 每次新终端启动前

```bash
source ~/craic/nav_real_ws/devel/setup.bash
```

## 3) 启动实机导航

推荐顺序见 `NAV_REAL_WS.md`：**先**实车底盘与传感器，**再**本工作空间导航栈（与底盘同一 ROS master）。

```bash
# 导航栈（二选一；激光直接用实车 /scan_filtered，无需单独起 topic_remap）

# AMCL + 静态地图（默认会打开 RViz，见第 4 节）
roslaunch car_sim nav_real_amcl.launch
# 或含相机 remap：
# roslaunch car_sim nav_real_amcl_with_remap.launch

# Hector 在线建图 + 定位 + move_base（默认会打开 RViz）
roslaunch car_sim nav_real_hector.launch
# 或含相机 remap：
# roslaunch car_sim nav_real_hector_with_remap.launch

# 无图形界面的小车 / SSH 无 DISPLAY 时关闭内置 RViz，在笔记本上单独开（见第 4.2 节）
# roslaunch car_sim nav_real_amcl.launch no_rviz:=true
# roslaunch car_sim nav_real_hector.launch no_rviz:=true

# 兼容旧命令（等同 nav_real_amcl*）
# roslaunch car_sim nav_real.launch
# roslaunch car_sim nav_real_with_remap.launch
```

地图文件：`src/car_sim/map/map_sim.yaml` + `map_sim.pgm`。在 `nav_sim_ws` 用 Hector 建图保存后同步到本目录（见 `nav_sim_ws/QUICKSTART.md` §3.1）；`hector.launch` 默认 `map_size:=200`，无需事后裁剪。

## 4) 局域网订阅小车 ROS 话题

仓库默认假设：**ROS master 在小车上**（底盘 launch 同机起 `roscore` 或等价 master）。笔记本/台式机通过设置环境变量订阅话题、开 RViz，无需把导航栈拷到电脑上跑 master。

代码同步到小车的脚本见仓库根目录 `sync_to_robot.sh`。**两台实车 IP**：1 号车 `192.168.124.3`（脚本默认）、2 号车 `192.168.124.9`。

### 4.1 小车上

```bash
export ROBOT_TYPE=EPRobotV2.3   # 按车型
roslaunch eprobot_chassis_bringup chassis.launch

# 可选：导航也在小车上起
source ~/craic/nav_real_ws/devel/setup.bash
roslaunch car_sim nav_real_amcl.launch no_rviz:=true
```

确认 master 与话题：

```bash
echo $ROS_MASTER_URI    # 常见为 http://<小车IP>:11311
rostopic list
rostopic hz /scan_filtered
```

### 4.2 你的电脑（与小车同一 WiFi / 局域网）

| 小车 | IP | 说明 |
|------|-----|------|
| 1 号车 | `192.168.124.3` | `sync_to_robot.sh` 默认；双车 TCP **server** |
| 2 号车 | `192.168.124.9` | `sync_to_robot.sh EPRobot@192.168.124.9`；双车 TCP **client** |

将 `<小车IP>` 换成上表地址（连 1 号车用 `192.168.124.3`，连 2 号车用 `192.168.124.9`），`<本机IP>` 换成你电脑在局域网内的 IP（不要用 `127.0.0.1`）：

```bash
export ROS_MASTER_URI=http://<小车IP>:11311
export ROS_IP=<本机IP>
# 或：export ROS_HOSTNAME=<本机主机名>

# 每个新终端都要重新 export（或写入 ~/.bashrc）
ping <小车IP>
rostopic list
rostopic echo /scan_filtered
rostopic echo /odom
rosrun tf tf_echo odom base_footprint
```

若 `rostopic list` 为空或没有数据：

- 检查 `ROS_MASTER_URI`、`ROS_IP` 是否配对正确
- 防火墙是否放行 **11311** 及 ROS 节点间通信（可先临时关闭防火墙排查）
- 小车底盘/雷达是否已启动

### 4.3 常用话题（与本仓库约定）

| 话题 | 说明 |
|------|------|
| `/scan_filtered` | 导航用激光（AMCL / Hector / costmap） |
| `/odom` | 里程计 |
| `/camera/rgb/image_raw` | 控制/视觉（与官方 `uvc_camera`、仿真 Gazebo 一致） |
| `/map` | 地图（map_server 或 Hector） |
| `/move_base/...` | 导航栈 |

相机话题已与官方对齐，一般无需 `topic_remap_ros`；legacy 说明见 `src/topic_remap_ros/README.md`。

### 4.4 拓扑说明

| 方式 | 说明 |
|------|------|
| **推荐** | master 在小车；笔记本只监控 / 开 RViz / 跑 `control_ws` |
| 笔记本当 master | 小车也需 `ROS_MASTER_URI` 指向笔记本；双方都要设对 `ROS_IP` |

## 5) 开启 RViz

预配置文件：`src/car_sim/rviz/nav.rviz`（代价地图、路径、激光等已按导航栈话题配置）。

### 5.1 随 launch 自动打开（默认）

`nav_real_amcl.launch` / `nav_real_hector.launch` 默认 **`no_rviz:=false`**，在本机有 `DISPLAY` 时会自动启动 RViz 并加载 `nav.rviz`。

```bash
roslaunch car_sim nav_real_amcl.launch
# 或
roslaunch car_sim nav_real_hector.launch
```

### 5.2 关闭 launch 内置 RViz，单独开

小车无桌面、SSH 无显示、或想在**笔记本**上看图时：

**小车或 SSH 终端：**

```bash
roslaunch car_sim nav_real_amcl.launch no_rviz:=true
```

**笔记本（已按第 4.2 节设置 `ROS_MASTER_URI` / `ROS_IP`）：**

```bash
source ~/craic/nav_real_ws/devel/setup.bash
export ROS_MASTER_URI=http://<小车IP>:11311
export ROS_IP=<本机IP>
rviz -d ~/craic/nav_real_ws/src/car_sim/rviz/nav.rviz
```

### 5.3 在 RViz 里常用操作

**AMCL**（`nav_real_amcl.launch`）：

- **Fixed Frame** 选 `map`
- 使用 **2D Pose Estimate** 设置初始位姿（`amcl.launch` 已默认 `initial_pose` 为 0,0,0 时，可微调）
- 使用 **2D Nav Goal** 下发导航目标点

**Hector**（`nav_real_hector.launch`）：

- **Fixed Frame** 选 `map`
- 先缓慢移动机器人，待 **Map** 与代价地图出现后再发目标
- 使用 **2D Nav Goal** 下发导航目标点
- 建图完成后保存：`roslaunch car_sim map_save.launch filename:=/path/to/my_map`

**仅查看传感器（未起导航时）：**

```bash
rviz
# 添加 LaserScan 显示，Topic 填 /scan 或 /scan_filtered
# Fixed Frame 可先选 odom 或 base_footprint
```

## 6) 最常改的参数文件

- TEB：`~/craic/nav_real_ws/src/car_sim/param/base_local_planner_params_TEB.yaml`
- Costmap：`~/craic/nav_real_ws/src/car_sim/param/costmap_common_params.yaml`

更多说明见 `NAV_REAL_WS.md`。控制节点见 `control_ws/README.md` 与 `~/craic/lh.txt` 中的任务点测试命令。

## 7) 实车性能基准与验收

以下基准在 **EPRobot V2.3、树莓派实机、底盘 + `nav_real_amcl`、地图 `map_sim`（200×200）** 上总结，用于判断导航栈是否「真的在跑」，而不是只看 RViz 里定位是否跟手。

### 7.1 正常时应看到的指标

| 项目 | 基准值 | 说明 |
|------|--------|------|
| **`/cmd_vel` 发布频率** | **约 7.5～8 Hz**（间隔 ~0.12～0.13 s） | 已发 **2D Nav Goal** 且 move_base 为 ACTIVE 时；静止或未导航时无输出是正常的 |
| **`controller_frequency`** | **8 Hz** | `move_base.launch` 配置；与上表一致即说明控制环未严重超时 |
| **`/odom`** | **约 50 Hz** | 底盘 `art_racecar` 发布 |
| **`/scan_filtered`** | 稳定有数据 | `frame_id` 应为 **`base_laser_link`** |
| **map_server** | 日志 `Received a 200 X 200 map` | `map_sim.yaml` 中 **`image: map_sim.pgm`**（相对路径），勿用 Docker 绝对路径 |
| **TEB 关键参数** | **`enable_homotopy_class_planning: false`**（必改） | yaml 未写时 TEB 默认 **true**，Pi 上易导致 `/cmd_vel` ~1Hz；**须重启 nav**，运行中 `rosparam set` 无效 |
| **TEB 其它** | craic F1 调参（`weight_optimaltime: 5`、`min_obstacle_dist: 0.04` 等） | 与官方 EPRobot yaml 不必完全一致；执行频率正常后再按需微调贴墙/速度 |

定位（AMCL）更新可以很快、看起来很准，但若 **`/cmd_vel` 只有约 1 Hz**，车仍会「极其慢」——**执行频率**比 **`max_vel_x` 上限**更要先验收。

### 7.2 快速验收命令

底盘与导航已启动，RViz **2D Pose Estimate** 后发 **2D Nav Goal**：

```bash
# 执行层（最重要）
rostopic hz /cmd_vel
rosparam get /move_base/controller_frequency
rosparam get /move_base/TebLocalPlannerROS/enable_homotopy_class_planning
# weight_optimaltime 等为 craic F1 值（默认 5），不必与官方一致

# 传感器与地图
rostopic hz /odom
rostopic hz /scan_filtered
grep image ~/craic/nav_real_ws/src/car_sim/map/map_sim.yaml

# 规划状态
rostopic echo /move_base/status -n 3
```

**通过参考**：`/cmd_vel` 平均 **≥ 6 Hz**、max 间隔 **< 0.5 s**；move_base 终端**很少**连续 `Control loop missed`（偶发可接受）。

### 7.3 常见异常与含义

| 现象 | 可能原因 |
|------|----------|
| `/cmd_vel` **~0.8～2 Hz**，间隔可达数秒 | TEB 默认开启多拓扑（yaml 未写 `enable_homotopy_class_planning: false`）、或 Pi 上 costmap/TEB 过重；见 move_base 日志 `Control loop missed` |
| `map_server` 找不到 `/root/craic/.../map_sim.pgm` | `map_sim.yaml` 仍为 Docker 路径；改为 `image: map_sim.pgm` 并 sync |
| 大量 `trajectory is not feasible` | 阿克曼转弯半径/通道宽度/障碍距离与地图不匹配；可对照官方 TEB 障碍参数或微调 costmap |
| 定位好但车不走 | 未发 Nav Goal、未做初始位姿、teleop 占用 `/cmd_vel`、或 move_base 非 ACTIVE |

### 7.4 跑圈 / 任务点测试

- **仅导航**：按 `~/craic/lh.txt` 顺序用 RViz **2D Nav Goal** 或 `rostopic pub /move_base_simple/goal` 走 `home → board1 → pickup_* → board2 → deliver_*`；坐标与 `control_ws` 中 `GOAL_LIST` 一致，换图后需在 RViz 中核对是否仍贴墙。
- **完整药房流程**：导航保持运行后，`roslaunch move_nav yaofang_service_mock.launch max_rounds:=1`（或 `control.launch`）；见 `control_ws/README.md`。

---

## Docker（仿真导航，可选）

Docker 镜像用于在**非 Ubuntu 18.04** 或不想本机装 Melodic 时跑 **Gazebo 仿真**（`nav_sim.launch`），不是用来替代实机 `nav_real.launch` 的。

完整步骤见 **[`nav_sim_ws/QUICKSTART.md` → 第 7 节 Docker](../nav_sim_ws/QUICKSTART.md)**。以下为常用命令摘要。

### 安装 Docker 与镜像加速

```bash
sudo apt update
sudo apt install -y docker.io docker-compose-v2
sudo usermod -aG docker $USER
newgrp docker   # 或注销后重新登录
```

国内拉取 Docker Hub 若报 `i/o timeout`，配置加速（示例 `docker.1ms.run`）：

```bash
sudo mkdir -p /etc/docker
sudo tee /etc/docker/daemon.json <<'EOF'
{
  "registry-mirrors": ["https://docker.1ms.run"]
}
EOF
sudo systemctl daemon-reload
sudo systemctl restart docker

docker info | grep -A3 "Registry Mirrors"
docker pull hello-world
```

### 构建与运行仿真

```bash
cd ~/craic
docker build -t craic:melodic .
xhost +local:docker
docker run --rm -it --net=host \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  craic:melodic bash
# 容器内：
source /root/craic/nav_sim_ws/devel/setup.bash
roslaunch car_sim nav_sim.launch
```
