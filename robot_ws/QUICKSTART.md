# robot_ws 快速启动

> 底盘架构与联调说明见 [README.md](README.md)。

## 实车终端 A — 底盘

```bash
export ROBOT_TYPE=EPRobotV2.3
source ~/robot_ws/devel/setup.bash
roslaunch eprobot_chassis_bringup chassis.launch
```

**无 EKF**（Hector 或 `nav_real_amcl_no_ekf`）时须发布 odom TF：

```bash
roslaunch eprobot_chassis_bringup chassis.launch pub_odom_tf:=true
```

## 首次部署

将 `robot_ws/src/eprobot_chassis_bringup` 拷到小车 `~/robot_ws/src`，与官方 `eprobot_start`、雷达包一起：

```bash
cd ~/robot_ws && catkin_make && source devel/setup.bash
```

## 验证

```bash
rostopic hz /scan_filtered
rostopic hz /camera/rgb/image_raw
rostopic hz /odom
```

导航与主控在同一 master 上启动 → [`../nav_real_ws/QUICKSTART.md`](../nav_real_ws/QUICKSTART.md)、[`../control_ws/QUICKSTART.md`](../control_ws/QUICKSTART.md)。
