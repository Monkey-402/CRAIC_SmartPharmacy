# nav_real_ws

实机导航工作空间，由 `nav_sim_ws` 派生，**默认 launch 不启动 Gazebo**。在实车 `robot_ws` 底盘与传感器就绪后，于同一 ROS master 上运行 `move_base + AMCL/Hector + TEB`。

**启动命令** → [`QUICKSTART.md`](QUICKSTART.md)

## 与 nav_sim_ws 的差异

| 项目 | nav_sim_ws | nav_real_ws |
|------|------------|-------------|
| 主入口 | `nav_sim.launch` / `nav_sim_amcl.launch` / `hector_sim.launch` | `nav_real_amcl.launch` / `nav_real_hector.launch` |
| 仿真时间 | `/clock`（Gazebo） | `use_sim_time:=false` |
| 话题桥接 | 无 | 可选 `topic_remap_ros`（默认全关） |
| 基坐标系 | `base_footprint` | `base_footprint`（对齐官方） |

包 `yaofang_world` 仍保留在工作空间内便于联编，**实机导航默认不引用**。

## 推荐栈顺序

1. **底盘**（`robot_ws`）：`chassis.launch`，默认 `pub_odom_tf:=false`  
2. **导航**（本工作空间）：`nav_real_amcl.launch`（EKF + AMCL）或 `nav_real_amcl_no_ekf` / `nav_real_hector`  
3. **主控**（`control_ws`）：订阅 `/camera/rgb/image_raw`、`/scan_filtered` 等  

> **对齐官方**：AMCL / costmap 使用 `base_footprint`；默认 AMCL 栈启用 **EKF**（`/odom` + `/imu_data` → `/odometry/filtered`）。TEB yaml **须**设 `enable_homotopy_class_planning: false`（Pi 上否则 `/cmd_vel` 可能仅 ~1 Hz）。

导航是否正常，优先看发 Nav Goal 后 **`/cmd_vel` 是否维持约 8 Hz** → [`QUICKSTART.md` §7](QUICKSTART.md#7-实车性能基准与验收)。

## Launch 入口

| 定位方式 | Launch | 说明 |
|----------|--------|------|
| AMCL + EKF（默认） | `nav_real_amcl.launch` | 别名 `nav_real.launch` |
| AMCL + EKF，双车 | `nav_real_amcl_car1.launch` / `car2.launch` | 初值 home / standby |
| AMCL，无 EKF | `nav_real_amcl_no_ekf.launch` | 底盘须 `pub_odom_tf:=true` |
| AMCL 无 EKF，双车 | `nav_real_amcl_no_ekf_car1/2.launch` | 同上 |
| Hector SLAM | `nav_real_hector.launch` | 在线建图 + 导航；底盘须 `pub_odom_tf:=true` |

带 `_with_remap` 的 launch 仍 include `topic_remap_ros`，但默认 **不转发**任何话题，行为与无 remap 版等价。

公共参数：`use_sim_time`（默认 `false`）、`no_rviz`（默认 `false`）、`teb_config`（TEB yaml 路径）、`map`（默认 `map_sim.yaml`）。

## 地图与参数

- 地图：`src/car_sim/map/map_sim.yaml` + `map_sim.pgm`（Hector 建图见 `nav_sim_ws/QUICKSTART` §3.1）。  
- TEB 四套预设、Costmap：`src/car_sim/param/`（与 `nav_sim_ws` 同名文件保持同步）。  
- RViz 配置：`src/car_sim/rviz/nav.rviz`。

## 常用话题

| 话题 | 说明 |
|------|------|
| `/scan_filtered` | 导航激光 |
| `/odom` / `/odometry/filtered` | 里程计（EKF 模式下后者为主） |
| `/camera/rgb/image_raw` | 相机（与底盘、主控一致） |
| `/move_base` | 导航 action |

Legacy 话题 remap → [`src/topic_remap_ros/README.md`](src/topic_remap_ros/README.md)。

## 依赖

与 `nav_sim_ws` 相同：`move_base`、`amcl`、`map_server`、`teb_local_planner`、`robot_localization`（EKF）等。不运行 Gazebo 时可不依赖 `gazebo_ros` 运行时。
