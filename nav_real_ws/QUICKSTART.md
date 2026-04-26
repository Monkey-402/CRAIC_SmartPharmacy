# Quickstart

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

```bash
roslaunch car_sim nav_sim.launch
```

## 4) 在 RViz 里常用操作

- 使用 `2D Pose Estimate` 设置初始位姿
- 使用 `2D Nav Goal` 下发导航目标点

## 5) 快速重启（参数改完后）

```bash
# 先 Ctrl+C 结束当前 roslaunch
roslaunch car_sim nav_sim.launch
```

## 6) 最常改的两个参数文件

- TEB：`~/craic/nav_sim_ws/src/car_sim/param/base_local_planner_params_TEB.yaml`
- Costmap：`~/craic/nav_sim_ws/src/car_sim/param/costmap_common_params.yaml`

