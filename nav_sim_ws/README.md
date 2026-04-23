# nav_sim_ws

`nav_sim_ws` 是一个面向开发与调参的 ROS 导航仿真工作区，核心目标是：

- 在 Gazebo 中复现室内药房场景（`yaofang`）
- 使用简化车体模型进行定位与路径规划验证
- 基于 `move_base + TEB` 快速迭代导航参数

---

## 软件架构

系统按 4 层组织：

1. **场景层（World/Map）**
   - Gazebo world: `src/yaofang_world/worlds/yaofang.world`
   - Gazebo model: `src/yaofang_world/models/yaofang/`
   - 静态栅格地图: `src/car_sim/map/map_sim.yaml` + `.pgm`

2. **机器人模型层（Robot Description）**
   - 简化车体与传感器定义: `src/robot_description/car_simple/urdf/car_simple.urdf`
   - 机器人由 `car_urdf.launch` 注入 `robot_description` 并在 Gazebo 中 spawn

3. **定位与导航层（Localization + Planning）**
   - 定位: `amcl`
   - 全局规划: `move_base` 默认全局规划器
   - 局部规划: `teb_local_planner/TebLocalPlannerROS`
   - 代价地图: global/local costmap

4. **可视化与交互层（Visualization）**
   - RViz: `src/car_sim/rviz/nav.rviz`
   - 典型交互：`2D Pose Estimate`、`2D Nav Goal`

---

## 包职责说明

- `src/car_sim`
  - 导航系统主入口，集中管理 launch、参数和地图
  - 启动链路：场景 -> 机器人 -> 定位 -> move_base -> rviz
- `src/robot_description/car_simple`
  - 轻量机器人模型，适合快速调参与避障行为验证
- `src/yaofang_world`
  - 仿真地形与贴图资源（包括地板贴图材质与墙体模型）

---

## 运行时节点与数据流

启动 `roslaunch car_sim nav_sim.launch` 后，关键节点关系为：

- Gazebo 发布仿真时间 `/clock`
- 机器人插件发布 `/odom`、`/scan`、`/imu/data`、相机图像
- `map_server` 发布静态地图
- `amcl` 融合 `/scan + map + odom`，输出 `map -> odom`
- `move_base` 读取代价地图与定位结果，输出 `/cmd_vel`
- 机器人底盘插件消费 `/cmd_vel` 驱动模型运动

这条闭环是调参核心：`/scan -> costmap/TEB -> /cmd_vel -> /odom -> AMCL/TF`。

---

## 目录结构（开发者视角）

- `src/car_sim/launch/nav_sim.launch`：总入口
- `src/car_sim/launch/car_urdf.launch`：world 与机器人 spawn
- `src/car_sim/launch/move_base.launch`：move_base 与 TEB 装载
- `src/car_sim/param/base_local_planner_params_TEB.yaml`：TEB 运动学/避障权重
- `src/car_sim/param/costmap_common_params.yaml`：footprint 与膨胀配置
- `src/yaofang_world/models/yaofang/model.sdf`：墙体/地板几何和材质引用
- `src/yaofang_world/models/yaofang/materials/`：贴图与材质脚本

---

## 构建与启动

```bash
cd ~/craic/nav_sim_ws
catkin_make
source devel/setup.bash
roslaunch car_sim nav_sim.launch
```

---

## 当前默认配置

- 局部规划器：`teb_local_planner/TebLocalPlannerROS`
- 机器人 footprint：`0.35m x 0.20m`（矩形）
- 地形：`yaofang.world`
- 机器人模型：`car_simple`

---

## 开发建议

- **调 TEB**：优先改 `min_obstacle_dist`、`inflation_dist`、`yaw_goal_tolerance`
- **调贴墙行为**：同步观察 TEB 参数与 `costmap_common_params.yaml` 的 `inflation_radius`

