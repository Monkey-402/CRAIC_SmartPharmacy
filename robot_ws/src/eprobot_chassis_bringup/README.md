# eprobot_chassis_bringup

本目录为 **`craic/robot_ws` 模板**：仅含一个底盘（含按车型启动雷达）的 launch，便于你**复制到实车 `~/catkin_ws/src`** 与现有 `eprobot_start` 等包一起使用。

## 依赖（实车上需已存在）

- `eprobot_start`（`art_racecar.py` 等）
- **RGB 相机（二选一）**
  - **默认** `astra_camera`（`roslaunch astra_camera astra.launch`，OpenNI2 → `/camera/rgb/image_raw`）
  - 可选 `uvc_camera` + `eprobot_start/astra_rgb_image.launch`（设备 `/dev/video0`，仅 UVC 正常时）
- `ROBOT_TYPE`：`EPRobotV2.2` 时需 `ls01d`；`EPRobotV2.3` 时需 `lslidar_driver`
- `eprobot_description`（`EPRobot_chassis.launch` 内引用）

## 使用

实车 IP：**1 号车 `192.168.124.3`**，**2 号车 `192.168.124.9`**。

```bash
export ROBOT_TYPE=EPRobotV2.3   # 或 EPRobotV2.2

# 默认：astra 相机（关深度、640x480@30Hz）+ pub_odom_tf=false
roslaunch eprobot_chassis_bringup chassis.launch

# 开启深度（按需）：
roslaunch eprobot_chassis_bringup chassis.launch camera_enable_depth:=true

# camera_color_mode（Astra.cfg）：5=640x480@30（默认），15=1280x960@7（需 patches/），3=1280x720@30
# roslaunch eprobot_chassis_bringup chassis.launch camera_color_mode:=15

# 出厂镜像 /dev/video0 可用时改回 UVC：
roslaunch eprobot_chassis_bringup chassis.launch camera_driver:=uvc

# 仅底盘+雷达、不启相机：
roslaunch eprobot_chassis_bringup chassis.launch enable_camera:=false

# 不用 EKF 导航时，底盘须发 odom TF：
roslaunch eprobot_chassis_bringup chassis.launch pub_odom_tf:=true
```

启动后相机话题为 **`/camera/rgb/image_raw`**，与 `control_ws` 默认订阅一致。

自检：`rostopic hz /camera/rgb/image_raw`

## 与 `craic/nav_real_ws` 联调

在同一 ROS master 上：先本包启动底盘与传感器，再：

- **EKF + AMCL（默认）**：`chassis.launch` → `nav_real_amcl.launch`
- **无 EKF**：`chassis.launch pub_odom_tf:=true` → `nav_real_amcl_no_ekf.launch`
- **Hector 建图**：`nav_real_hector.launch`（底盘 `pub_odom_tf:=true`）

## 说明

`EPRobot_chassis.launch` 含底盘与按 `ROBOT_TYPE` 的雷达；`chassis.launch` 再按 `camera_driver` 启 RGB。**勿同时**手动 `astra_camera` 与 `camera_driver:=uvc`。**不包含** `move_base`，导航请用 `nav_real_ws`。
