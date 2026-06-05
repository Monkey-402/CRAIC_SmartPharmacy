# Quickstart

> **总览**：[`../QUICKSTART.md`](../QUICKSTART.md)。架构见 [`README.md`](README.md)。

## 1) 一次性准备

```bash
cd ~/craic/nav_sim_ws
catkin_make
```

## 2) 每次新终端启动前

```bash
source ~/craic/nav_sim_ws/devel/setup.bash
```

## 3) 启动仿真导航

**与实车话题对齐（推荐，含 EKF + AMCL）：**

```bash
roslaunch car_sim nav_sim_amcl.launch
```

话题与 `nav_real_amcl` 一致：`/scan_filtered`、`/imu_data`、`/odometry/filtered`、`base_laser_link`、`IMU_link`。

**轻量 AMCL（无 EKF，仍走对齐后的话题名）：**

```bash
roslaunch car_sim nav_sim.launch
```

### 3.1) Hector 建图（生成实机用 map_sim）

默认 **200×200** 栅格（`hector.launch` 的 `map_size`，0.05m 下约 10m×10m）：

```bash
roslaunch car_sim hector_sim.launch
# 键盘 teleop 走遍场地后：
roslaunch car_sim map_save.launch
cp src/car_sim/map/map_sim.{pgm,yaml} ../nav_real_ws/src/car_sim/map/
```

场地更大时可 `roslaunch car_sim hector_sim.launch map_size:=256`。

导航栈坐标系已对齐官方：**`base_footprint`**（与 `car_simple.urdf`、实车 `EPRobot_start` 一致）。

`sim_sensor_bridge.launch` 将 Gazebo 的 `/scan`、`/imu/data` 对齐为实车的 `/scan_filtered`、`/imu_data`，并发布 `laser_link→base_laser_link`、`imu_link→IMU_link` 静态 TF。

药房主控（仿真专用 launch，放宽 QR 检测）→ [`control_ws/QUICKSTART.md`](../control_ws/QUICKSTART.md) §4。

## 4) 在 RViz 里常用操作

- 使用 `2D Pose Estimate` 设置初始位姿
- 使用 `2D Nav Goal` 下发导航目标点

## 5) 快速重启（参数改完后）

```bash
# 先 Ctrl+C 结束当前 roslaunch
roslaunch car_sim nav_sim.launch
```

## 6) TEB 参数预设与启动

与实车相同四套预设，目录：`~/craic/nav_sim_ws/src/car_sim/param/`。**预设说明与验证命令**见 **[`nav_real_ws/QUICKSTART.md` §6](../nav_real_ws/QUICKSTART.md)**（权威版，避免两处维护）。

| 文件 | 说明 |
|------|------|
| `base_local_planner_params_TEB.yaml` | 默认保守 |
| `base_local_planner_params_TEB_smooth.yaml` | 顺滑 + 提速（`max_vel_x` 1.15，与实车同版） |
| `base_local_planner_params_TEB_conservative_half.yaml` | 一半速度 |
| `base_local_planner_params_TEB_official_max_vel.yaml` | 官方最大速度 |

```bash
source ~/craic/nav_sim_ws/devel/setup.bash

roslaunch car_sim nav_sim.launch \
  teb_config:=$(rospack find car_sim)/param/base_local_planner_params_TEB_smooth.yaml

roslaunch car_sim nav_sim_amcl.launch \
  teb_config:=$(rospack find car_sim)/param/base_local_planner_params_TEB_conservative_half.yaml
```

Costmap：`costmap_common_params.yaml`。

## 7) Docker（Ubuntu 18.04 + ROS Melodic）

宿主机无 Melodic 时用容器跑 Gazebo 仿真。**安装、构建、运行、排错** 均在 **[`docker/`](../docker/)** 目录：

- [docker/README.md](../docker/README.md) — 镜像内容与适用场景  
- [docker/QUICKSTART.md](../docker/QUICKSTART.md) — 命令步骤  

```bash
cd ~/craic/docker
xhost +local:docker
docker compose build
docker compose run --rm craic roslaunch car_sim nav_sim_amcl.launch
```

