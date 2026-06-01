# craic/robot_ws（实车底盘模板）

仅包含 **`eprobot_chassis_bringup`**：启动 EPRobot 底盘、按 `ROBOT_TYPE` 选择雷达，默认 **`astra_camera`**（关深度、640×480 RGB → `/camera/rgb/image_raw`）；可选 `camera_driver:=uvc`。

**实车 IP**：1 号车 `192.168.124.3`，2 号车 `192.168.124.9`（`sync_to_robot.sh` 默认同步到 1 号车）。

## 用法

将整个 `robot_ws` 复制到实车，或只把 `src/eprobot_chassis_bringup` 拷贝到现有 `~/catkin_ws/src`，与主仓库中的 `eprobot_start`、`lslidar_driver` / `ls01d` 等一起 `catkin_make` 后：

```bash
export ROBOT_TYPE=EPRobotV2.3
roslaunch eprobot_chassis_bringup chassis.launch
```

## 与 `nav_real_ws` 联调

底盘在本机起好后，再在同一 ROS master 上启动 `nav_real_ws` 的 `nav_real_hector.launch` 或 `nav_real_amcl.launch`（激光 `/scan_filtered`）。`control_ws` 与底盘均使用 **`/camera/rgb/image_raw`**，无需相机 topic remap。

## Catkin 工作空间

`src/CMakeLists.txt` 已链到本机 ROS Noetic 的 catkin 顶层；若你使用 Melodic，请将该 symlink 改为对应发行版的 `toplevel.cmake`。
