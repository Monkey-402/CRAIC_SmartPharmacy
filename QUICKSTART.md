# CRAIC 快速上手

只列**命令**；说明、参数表、依赖见 [`README.md`](README.md) 与各子目录文档。

---

## 实车

实车操作需先 SSH 登录（密码 `ncut1234`）：

```bash
ssh EPRobot@192.168.124.3   # 1 号
ssh EPRobot@192.168.124.9   # 2 号
```

```bash
# 开发机同步代码
cd ~/craic
./sync_to_robot.sh                         # 1 号
./sync_to_robot.sh EPRobot@192.168.124.9  # 2 号
```

同步后在小车 SSH 里编译（有改动时执行）：

```bash
cd ~/craic/nav_real_ws && catkin_make && source devel/setup.bash
cd ~/craic/control_ws && catkin_make && source devel/setup.bash
cd ~/robot_ws && catkin_make && source devel/setup.bash
```

**终端 A～D** 在对应小车的 SSH 里执行。

**终端 A — 底盘**

```bash
export ROBOT_TYPE=EPRobotV2.3
source ~/robot_ws/devel/setup.bash
roslaunch eprobot_chassis_bringup chassis.launch
```

**终端 B — 导航**

```bash
source ~/craic/nav_real_ws/devel/setup.bash
# 1 号：
roslaunch car_sim nav_real_amcl_car1.launch no_rviz:=true teb_config:=$(rospack find car_sim)/param/base_local_planner_params_TEB_official_max_vel.yaml
# 2 号：
roslaunch car_sim nav_real_amcl_car2.launch no_rviz:=true teb_config:=$(rospack find car_sim)/param/base_local_planner_params_TEB_official_max_vel.yaml
# 顺滑 TEB：追加 teb_config:=$(rospack find car_sim)/param/base_local_planner_params_TEB_smooth.yaml
# 其它 teb_config / no_ekf / hector：见 README.md「TEB 参数预设」与 nav_real_ws/QUICKSTART §6
```

**终端 C — 主控**

```bash
source ~/craic/control_ws/devel/setup.bash
# 1 号：
roslaunch move_nav real_car1.launch use_paddle_ocr:=true
# 2 号：
roslaunch move_nav real_car2.launch use_paddle_ocr:=true

# 单轮：yaofang_service_mock.launch max_rounds:=1
# Paddle：先终端 D ./run_paddle_ocr_server.sh，再 real_car1.launch use_paddle_ocr:=true
```

**终端 D — Paddle HTTP**

```bash
source ~/craic/control_ws/devel/setup.bash
roscd board2_paddle_ocr && ./run_paddle_ocr_server.sh
```

<!-- **本机 RViz**

```bash
export ROS_MASTER_URI=http://192.168.124.3:11311   # 或 .9
export ROS_IP=<本机 IP>
source ~/craic/nav_real_ws/devel/setup.bash
rviz -d ~/craic/nav_real_ws/src/car_sim/rviz/nav.rviz
``` -->

<!-- **验收**

```bash
rostopic hz /cmd_vel
rostopic hz /scan_filtered
rosparam get /move_base/TebLocalPlannerROS/enable_homotopy_class_planning
``` -->

---

## TEB 切换（重启导航生效）

```bash
source ~/craic/nav_real_ws/devel/setup.bash

roslaunch car_sim nav_real_amcl_car1.launch no_rviz:=true \
  teb_config:=$(rospack find car_sim)/param/base_local_planner_params_TEB_smooth.yaml

roslaunch car_sim nav_real_amcl_car2.launch no_rviz:=true \
  teb_config:=$(rospack find car_sim)/param/base_local_planner_params_TEB_conservative_half.yaml

roslaunch car_sim nav_real_amcl.launch no_rviz:=true \
  teb_config:=$(rospack find car_sim)/param/base_local_planner_params_TEB_official_max_vel.yaml
```

```bash
source ~/craic/nav_sim_ws/devel/setup.bash
roslaunch car_sim nav_sim_amcl.launch \
  teb_config:=$(rospack find car_sim)/param/base_local_planner_params_TEB_smooth.yaml
```

预设含义与验证 → [`README.md`](README.md)；更多 launch → [`nav_real_ws/QUICKSTART.md`](nav_real_ws/QUICKSTART.md) §6。

---
