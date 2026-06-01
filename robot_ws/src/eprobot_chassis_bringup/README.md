# eprobot_chassis_bringup

本目录为 **`craic/robot_ws` 模板**：仅含一个底盘（含按车型启动雷达）的 launch，便于你**复制到实车 `~/catkin_ws/src`** 与现有 `eprobot_start` 等包一起使用。

## 依赖（实车上需已存在）

- `eprobot_start`（`art_racecar.py` 等）
- `uvc_camera`（`astra_rgb_image.launch` 内 `uvc_camera_node`，设备默认 `/dev/video0`）
- `ROBOT_TYPE`：`EPRobotV2.2` 时需 `ls01d`；`EPRobotV2.3` 时需 `lslidar_driver`
- `eprobot_description`（`EPRobot_start.launch` 内引用）

## 使用

实车 IP：**1 号车 `192.168.124.3`**，**2 号车 `192.168.124.9`**。

```bash
export ROBOT_TYPE=EPRobotV2.3   # 或 EPRobotV2.2
roslaunch eprobot_chassis_bringup chassis.launch
# 仅底盘+雷达、不启相机：enable_camera:=false
```

启动后相机话题为 **`/camera/rgb/image_raw`**，与 `control_ws` 默认订阅一致。

## 与 `craic/nav_real_ws` 联调

在同一 ROS master 上：先本包启动底盘与传感器，再 `car_sim/nav_real_hector.launch` 或 `nav_real_amcl.launch`。

## 说明

`EPRobot_start.launch` 已包含底盘与按 `ROBOT_TYPE` 的雷达；`chassis.launch` 另 include 官方 `astra_rgb_image.launch`（等同单独 `roslaunch eprobot_start astra_rgb_image.launch`）。**不包含** `move_base`，导航请用 `nav_real_ws`。
