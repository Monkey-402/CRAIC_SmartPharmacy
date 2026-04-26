# craic/robot_ws（实车底盘模板）

仅包含 **`eprobot_chassis_bringup`**：一个用于启动 EPRobot 底盘（及按 `ROBOT_TYPE` 选择雷达）的 launch。

## 用法

将整个 `robot_ws` 复制到实车，或只把 `src/eprobot_chassis_bringup` 拷贝到现有 `~/catkin_ws/src`，与主仓库中的 `eprobot_start`、`lslidar_driver` / `ls01d` 等一起 `catkin_make` 后：

```bash
export ROBOT_TYPE=EPRobotV2.3
roslaunch eprobot_chassis_bringup chassis.launch
```

## 与 `nav_real_ws` 联调

底盘在本机起好后，再在同一 ROS master 上启动 `nav_real_ws` 的 `topic_remap_ros` 与 `nav_real.launch`（或 `nav_real_with_remap.launch`）。

## Catkin 工作空间

`src/CMakeLists.txt` 已链到本机 ROS Noetic 的 catkin 顶层；若你使用 Melodic，请将该 symlink 改为对应发行版的 `toplevel.cmake`。
