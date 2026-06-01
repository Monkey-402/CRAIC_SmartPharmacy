# nav_real_ws（实机导航工作空间）

由 `nav_sim_ws` 复制而来，**默认入口不再启动 Gazebo**。

## 与 `nav_sim_ws` 的差异

| 项目 | nav_sim_ws | nav_real_ws |
|------|------------|-------------|
| 主入口 | `nav_sim.launch` / `hector_sim.launch`（建图） | `nav_real_amcl.launch` / `nav_real_hector.launch` |
| 话题桥接 | 无 | 可选 `topic_remap_ros`（默认全关；legacy 相机/scan 转发） |
| 机器人基坐标 | `base_footprint` | `base_footprint`（对齐官方） |

仿真世界包 `yaofang_world` 仍保留在工作空间内，但**实机导航默认 launch 不会引用它**。

## 推荐启动顺序（与实车 `robot_ws` 同机、同一 master）

1. 实车：`export ROBOT_TYPE=EPRobotV2.3` → `roslaunch eprobot_chassis_bringup chassis.launch`（默认 `pub_odom_tf:=false`，EKF 单独发 odom TF）
2. 导航：`roslaunch car_sim nav_real_amcl.launch`（EKF + AMCL）或 `nav_real_amcl_no_ekf.launch`（无 EKF，底盘须 `pub_odom_tf:=true`）或 `nav_real_hector.launch`（Hector，底盘须 `pub_odom_tf:=true`）
3. 控制端：`control_ws`（订阅 `/camera/rgb/image_raw`，与 `chassis.launch` 相机一致）

> **P0 对齐官方（当前）**：AMCL / costmap / Hector 使用 `base_footprint`；实车 `nav_real_amcl` 已启用 **EKF**（`/odom` + `/imu_data` → `/odometry/filtered`）；TEB **仅强制** `enable_homotopy_class_planning: false`，其余为 craic F1 调参。

实车导航是否正常，优先看 **`/cmd_vel` 是否在发目标后维持约 8 Hz**（详见 [`QUICKSTART.md` §7 性能基准](./QUICKSTART.md#7-实车性能基准与验收)）。

## Launch 入口

| 定位方式 | 单独启动 | 兼容别名（含 topic_remap，默认不转发） |
|----------|----------|--------------------------------------|
| AMCL + EKF（默认） | `nav_real_amcl.launch` | `nav_real_amcl_with_remap.launch` |
| AMCL，无 EKF | `nav_real_amcl_no_ekf.launch` | — |
| Hector SLAM | `nav_real_hector.launch` | `nav_real_hector_with_remap.launch` |

`nav_real.launch` / `nav_real_with_remap.launch` 为兼容别名，等同 AMCL 版本。

公共参数：`use_sim_time`（默认 `false`）、`no_rviz`（默认 `false`）。  
AMCL 另支持 `map`（默认 `map_sim.yaml`，位于 `car_sim/map/`）。

示例：

```bash
roslaunch car_sim nav_real_amcl.launch map:=map_sim.yaml no_rviz:=true
roslaunch car_sim nav_real_hector.launch no_rviz:=true
```

## 依赖

与 `nav_sim_ws` 相同：`move_base`、`amcl`、`map_server`、`teb_local_planner` 等；**不包含** `gazebo_ros` 的运行时依赖即可单独跑 `nav_real.launch`（若仍编译 `yaofang_world`，需本机有 Gazebo 相关依赖）。
