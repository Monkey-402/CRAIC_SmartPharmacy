# nav_real_ws（实机导航工作空间）

由 `nav_sim_ws` 复制而来，**默认入口不再启动 Gazebo**。

## 与 `nav_sim_ws` 的差异

| 项目 | nav_sim_ws | nav_real_ws |
|------|------------|-------------|
| 主入口 | `nav_sim.launch` → 含 `car_urdf.launch`（Gazebo + spawn） | `nav_real_amcl.launch` / `nav_real_hector.launch` + move_base + RViz（可选） |
| 话题桥接 | 无 | 可选 `topic_remap_ros`（默认仅相机）；激光直接用 `/scan_filtered` |

仿真世界包 `yaofang_world` 仍保留在工作空间内，但**实机导航默认 launch 不会引用它**。

## 推荐启动顺序（与实车 `robot_ws` 同机、同一 master）

1. 实车：底盘 + 雷达 + TF +（可选）相机  
2. `roslaunch car_sim nav_real_amcl.launch` 或 `nav_real_hector.launch`  
3. 需相机 remap 时用 `nav_real_*_with_remap.launch`  
4. 控制端：`control_ws` 等（按需）

## Launch 入口

| 定位方式 | 单独启动 | 含相机 topic_remap |
|----------|----------|-------------------|
| AMCL + 静态地图 | `nav_real_amcl.launch` | `nav_real_amcl_with_remap.launch` |
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
