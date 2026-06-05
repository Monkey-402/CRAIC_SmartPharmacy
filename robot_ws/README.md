# craic/robot_ws（实车底盘模板）

仅包含 **`eprobot_chassis_bringup`**：启动 EPRobot 底盘、按 `ROBOT_TYPE` 选择雷达，默认 **`astra_camera`**（640×480@30Hz RGB → `/camera/rgb/image_raw`）；可选 `camera_driver:=uvc`。

**启动命令** → [`QUICKSTART.md`](QUICKSTART.md)

## 与导航 / 主控的关系

底盘在本机起好后，于**同一 ROS master** 启动：

- `nav_real_ws`：`nav_real_amcl.launch` 等（激光 `/scan_filtered`）  
- `control_ws`：主控与视觉（相机 `/camera/rgb/image_raw`）

默认 **`pub_odom_tf:=false`**，由 `nav_real_amcl` 内 EKF 发布 `odom→base_footprint`。无 EKF 栈时底盘须 `pub_odom_tf:=true`。

## Catkin

`src/CMakeLists.txt` 链到本机 ROS 的 catkin 顶层；Melodic / Noetic 需对应修改 symlink。

## 实车 IP

1 号 `192.168.124.3`，2 号 `192.168.124.9`（`sync_to_robot.sh` 默认同步到 1 号）。
