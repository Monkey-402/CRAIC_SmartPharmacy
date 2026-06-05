# CRAIC 快速上手

只列**实车全栈命令**；架构与参数见 [`README.md`](README.md) 与各工作空间 README。

| 场景 | 文档 |
|------|------|
| 本机仿真 / 建图 | [`nav_sim_ws/QUICKSTART.md`](nav_sim_ws/QUICKSTART.md) |
| Melodic 容器仿真 | [`docker/QUICKSTART.md`](docker/QUICKSTART.md) |
| 实车导航 / TEB / 验收 | [`nav_real_ws/QUICKSTART.md`](nav_real_ws/QUICKSTART.md) |
| 主控 / 视觉 / 双车 | [`control_ws/QUICKSTART.md`](control_ws/QUICKSTART.md) |
| 底盘 | [`robot_ws/QUICKSTART.md`](robot_ws/QUICKSTART.md) |

---

## 实车

SSH 登录（密码 `ncut1234`）：

```bash
ssh EPRobot@192.168.124.3   # 1 号
ssh EPRobot@192.168.124.9   # 2 号
```

**开发机同步代码：**

```bash
cd ~/craic
./sync_to_robot.sh                         # 1 号
./sync_to_robot.sh EPRobot@192.168.124.9  # 2 号
```

**小车编译（有改动时）：**

```bash
cd ~/craic/nav_real_ws && catkin_make && source devel/setup.bash
cd ~/craic/control_ws && catkin_make && source devel/setup.bash
cd ~/robot_ws && catkin_make && source devel/setup.bash
```

**终端 A — 底盘** → 见 [`robot_ws/QUICKSTART.md`](robot_ws/QUICKSTART.md)

**终端 B — 导航** → 见 [`nav_real_ws/QUICKSTART.md`](nav_real_ws/QUICKSTART.md) §3、§6（双车用 `nav_real_amcl_car1/2`）

**终端 C — 主控** → 见 [`control_ws/QUICKSTART.md`](control_ws/QUICKSTART.md)（`real_car1.launch` / `real_car2.launch`）

**终端 D — Paddle OCR（可选）** → 见 [`control_ws/QUICKSTART.md`](control_ws/QUICKSTART.md) §Paddle

---

## TEB 预设切换

完整预设表、验证命令与临时改参 → **[`nav_real_ws/QUICKSTART.md` §6](nav_real_ws/QUICKSTART.md)**。

常用示例（**须重启导航 launch**）：

```bash
source ~/craic/nav_real_ws/devel/setup.bash
roslaunch car_sim nav_real_amcl_car1.launch no_rviz:=true \
  teb_config:=$(rospack find car_sim)/param/base_local_planner_params_TEB_smooth.yaml
```

仿真同理 → [`nav_sim_ws/QUICKSTART.md` §6](nav_sim_ws/QUICKSTART.md)。
