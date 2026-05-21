# Quickstart（实机导航）

实机默认**不启 Gazebo**，在宿主 ROS（常见为 Noetic/Melodic）上运行。仿真调试见 `nav_sim_ws/QUICKSTART.md`。

## 1) 一次性准备

```bash
cd ~/craic/nav_real_ws
catkin_make
# 若与 nav_sim_ws 联编，先 source nav_sim_ws 再编译本工作空间
```

## 2) 每次新终端启动前

```bash
source ~/craic/nav_real_ws/devel/setup.bash
```

## 3) 启动实机导航

底盘与雷达由实车 `robot_ws` / `eprobot_start` 等提供后：

```bash
# 话题对齐（可选，见 topic_remap_ros）
roslaunch topic_remap_ros topic_remap_default.launch

# 导航栈
roslaunch car_sim nav_real.launch
# 或一键：roslaunch car_sim nav_real_with_remap.launch
```

## 4) 在 RViz 里常用操作

- 使用 `2D Pose Estimate` 设置初始位姿
- 使用 `2D Nav Goal` 下发导航目标点

## 5) 最常改的参数文件

- TEB：`~/craic/nav_real_ws/src/car_sim/param/base_local_planner_params_TEB.yaml`
- Costmap：`~/craic/nav_real_ws/src/car_sim/param/costmap_common_params.yaml`

更多说明见 `NAV_REAL_WS.md`。

---

## Docker（仿真导航，可选）

Docker 镜像用于在**非 Ubuntu 18.04** 或不想本机装 Melodic 时跑 **Gazebo 仿真**（`nav_sim.launch`），不是用来替代实机 `nav_real.launch` 的。

完整步骤见 **[`nav_sim_ws/QUICKSTART.md` → 第 7 节 Docker](../nav_sim_ws/QUICKSTART.md)**。以下为常用命令摘要。

### 安装 Docker 与镜像加速

```bash
sudo apt update
sudo apt install -y docker.io docker-compose-v2
sudo usermod -aG docker $USER
newgrp docker   # 或注销后重新登录
```

国内拉取 Docker Hub 若报 `i/o timeout`，配置加速（示例 `docker.1ms.run`）：

```bash
sudo mkdir -p /etc/docker
sudo tee /etc/docker/daemon.json <<'EOF'
{
  "registry-mirrors": ["https://docker.1ms.run"]
}
EOF
sudo systemctl daemon-reload
sudo systemctl restart docker

docker info | grep -A3 "Registry Mirrors"
docker pull hello-world
```

### 构建与运行仿真

```bash
cd ~/craic
docker build -t craic:melodic .
xhost +local:docker
docker run --rm -it --net=host \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  craic:melodic bash
# 容器内：
source /root/craic/nav_sim_ws/devel/setup.bash
roslaunch car_sim nav_sim.launch
```
