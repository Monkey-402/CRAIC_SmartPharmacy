# Docker 快速启动

> 镜像说明见 [README.md](README.md)。仿真 launch 见 [nav_sim_ws/QUICKSTART.md](../nav_sim_ws/QUICKSTART.md)。

## 1) 安装 Docker

```bash
sudo apt update
sudo apt install -y docker.io docker-compose-v2
sudo usermod -aG docker $USER
newgrp docker   # 或注销后重新登录
```

国内拉取 Hub 超时时配置镜像加速（示例 `docker.1ms.run`，失效可换其它源）：

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
docker pull hello-world && docker run --rm hello-world
```

> 若已有 `daemon.json`，请**合并** `registry-mirrors`，勿直接覆盖。

## 2) 构建镜像

```bash
cd ~/craic/docker
docker compose build
```

或不用 compose：

```bash
cd ~/craic
docker build -f docker/Dockerfile -t craic:melodic .
```

## 3) GUI（Gazebo / RViz）

```bash
xhost +local:docker
# 若 libGL / amdgpu 报错:
# export LIBGL_ALWAYS_SOFTWARE=1
```

## 4) 进入容器

**推荐（compose，自动挂载源码）：**

```bash
cd ~/craic/docker
xhost +local:docker
docker compose run --rm craic bash
```

容器内 entrypoint 已 source 工作空间，可直接：

```bash
roslaunch car_sim nav_sim_amcl.launch
```

**一条命令启动仿真：**

```bash
cd ~/craic/docker
docker compose run --rm craic roslaunch car_sim nav_sim_amcl.launch
```

**药房主控：**

```bash
docker compose run --rm craic bash -lc \
  'source /root/craic/control_ws/devel/setup.bash && roslaunch move_nav control_sim.launch'
```

## 5) 改代码后

已挂载 `~/craic` → `/root/craic` 时，容器内重新编译即可：

```bash
cd /root/craic/nav_sim_ws && catkin_make
source /root/craic/nav_sim_ws/devel/setup.bash
```

未挂载卷、仅用镜像内快照时，宿主机改代码后须重新 `docker compose build`。

## 6) 常见问题

| 现象 | 处理 |
|------|------|
| `docker: command not found` | 安装 `docker.io`，用户加入 `docker` 组 |
| 拉镜像 `i/o timeout` | 配置镜像加速（§1） |
| `car_sim` 找不到 | `source /root/craic/nav_sim_ws/devel/setup.bash` |
| RViz/Gazebo 无窗口 | `xhost +local:docker`、`-e DISPLAY`、挂载 `/tmp/.X11-unix` |
| SDF / Unicode 报错 | 更新仓库后重新 `docker compose build` |
