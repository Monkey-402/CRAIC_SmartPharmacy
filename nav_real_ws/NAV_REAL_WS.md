# nav_real_ws（实机导航工作空间）

由 `nav_sim_ws` 复制而来，**默认入口不再启动 Gazebo**。

## 与 `nav_sim_ws` 的差异

| 项目 | nav_sim_ws | nav_real_ws |
|------|------------|-------------|
| 主入口 | `nav_sim.launch` → 含 `car_urdf.launch`（Gazebo + spawn） | `nav_real.launch` → 仅 map_server + AMCL + move_base + RViz（可选） |
| 话题桥接 | 无 | `topic_remap_ros`：实车话题名对齐 craic 控制节点 |

仿真世界包 `yaofang_world` 仍保留在工作空间内，但**实机导航默认 launch 不会引用它**。

## 推荐启动顺序（与实车 `robot_ws` 同机、同一 master）

1. 实车：底盘 + 雷达 + TF +（可选）相机  
2. `roslaunch topic_remap_ros topic_remap_default.launch`  
3. `roslaunch car_sim nav_real.launch`  
4. 控制端：`control_ws` 等（按需）

## Launch 参数

- `nav_real.launch`  
  - `map`：地图 yaml 文件名，默认 `map_sim.yaml`（位于 `car_sim/map/`）  
  - `use_sim_time`：默认 `false`  
  - `no_rviz`：默认 `false`，设为 `true` 可不启 RViz  

示例：

```bash
roslaunch car_sim nav_real.launch map:=map_sim.yaml no_rviz:=true
```

一键（含话题桥接）：

```bash
roslaunch car_sim nav_real_with_remap.launch
```

## 依赖

与 `nav_sim_ws` 相同：`move_base`、`amcl`、`map_server`、`teb_local_planner` 等；**不包含** `gazebo_ros` 的运行时依赖即可单独跑 `nav_real.launch`（若仍编译 `yaofang_world`，需本机有 Gazebo 相关依赖）。
