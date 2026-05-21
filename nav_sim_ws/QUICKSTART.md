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

## 7) Docker（Ubuntu 18.04 + ROS Melodic）

在宿主机（如 Pop!_OS / Ubuntu 24.04）上用容器跑仿真，无需本机安装 Melodic。镜像定义在仓库根目录 `craic/Dockerfile`。

### 7.1) 安装 Docker 与镜像加速

```bash
sudo apt update
sudo apt install -y docker.io docker-compose-v2
sudo usermod -aG docker $USER
newgrp docker   # 或注销后重新登录
```

国内直连 Docker Hub 常出现 `i/o timeout`，建议配置镜像加速。下面以第三方加速 `docker.1ms.run` 为例（地址可能变动，失效时可搜索「Docker 镜像加速」替换）：

```bash
sudo mkdir -p /etc/docker
sudo tee /etc/docker/daemon.json <<'EOF'
{
  "registry-mirrors": ["https://docker.1ms.run"]
}
EOF
sudo systemctl daemon-reload
sudo systemctl restart docker
```

确认已生效：

```bash
docker info | grep -A3 "Registry Mirrors"
```

应能看到 `https://docker.1ms.run/`。再测拉取：

```bash
docker pull hello-world
docker run --rm hello-world
```

> 若已有 `/etc/docker/daemon.json`（例如配过代理），请把 `registry-mirrors` **合并**进同一 JSON，不要直接覆盖丢配置。

### 7.2) 构建镜像

```bash
cd ~/craic    # 或你的 craic 仓库路径
docker build -t craic:melodic .
```

可选：同时编译 `control_ws`（体积更大、耗时更长）：

```bash
docker build -t craic:melodic --build-arg BUILD_CONTROL_WS=1 .
```

### 7.3) 允许 GUI（Gazebo / RViz）

```bash
xhost +local:docker
```

若容器内出现 `libGL` / `amdgpu` 相关报错，可在启动前加：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

### 7.4) 进入容器终端

```bash
docker run --rm -it \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  craic:melodic bash
```

容器内工作空间已编译在 `/root/craic/`。启动仿真前执行：

```bash
source /root/craic/nav_sim_ws/devel/setup.bash
roslaunch car_sim nav_sim.launch
```

> 若 `rospack find car_sim` 失败，说明未 source 上述 `setup.bash`。

也可一条命令直接启动（entrypoint 会自动 source 工作空间）：

```bash
docker run --rm -it \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  craic:melodic \
  roslaunch car_sim nav_sim.launch
```

### 7.5) 使用 docker compose

```bash
cd ~/craic
xhost +local:docker
docker compose build
docker compose run --rm craic bash
# 或
docker compose run --rm craic roslaunch car_sim nav_sim.launch
```

### 7.6) 修改代码后

容器内源码在 `/root/craic/`。在宿主机改完代码后需**重新构建镜像**：

```bash
cd ~/craic
docker build -t craic:melodic .
```

### 7.7) 常见问题

| 现象 | 处理 |
|------|------|
| `docker: command not found` | 安装 `docker.io`，并将用户加入 `docker` 组 |
| 拉镜像 `i/o timeout` | 配置 `registry-mirrors`（见 7.1） |
| `car_sim` 找不到 | `source /root/craic/nav_sim_ws/devel/setup.bash` |
| RViz/Gazebo 无窗口 | 检查 `xhost +local:docker` 与 `-e DISPLAY`、挂载 `/tmp/.X11-unix` |
| `UnicodeEncodeError` / SDF 1.7 | 请使用当前仓库最新版（URDF 已去中文注释，`yaofang` 已改为 SDF 1.6）并重新 `docker build` |

