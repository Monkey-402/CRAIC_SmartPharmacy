# Docker 仿真环境

在 **Ubuntu 18.04 + ROS Melodic** 容器内跑 Gazebo 仿真与 `control_ws`，供宿主机（如 Ubuntu 22.04/24.04）无需本机安装 Melodic 时使用。

**快速启动** → [`QUICKSTART.md`](QUICKSTART.md)

## 目录

| 文件 | 说明 |
|------|------|
| `Dockerfile` | 镜像定义（Melodic + 导航栈 + 四工作空间 catkin_make） |
| `docker-compose.yml` | 推荐启动方式（挂载源码、X11、host 网络） |
| `entrypoint.sh` | 自动 source 各工作空间 overlay |
| `QUICKSTART.md` | 安装 Docker、构建、运行、排错 |

构建上下文为 **`craic/` 根目录**（`COPY` 整个仓库）。根目录的 [`.dockerignore`](../.dockerignore) 因 Docker 机制须放在上下文根，不在本目录重复维护。

## 镜像内容

- 基础：`osrf/ros:melodic-desktop-full`（含 Gazebo、RViz）
- 编译：`nav_sim_ws` → `nav_real_ws` → `robot_ws` → `control_ws`
- 视觉：Python 2 的 Tesseract、pyzbar（与实车 Melodic 一致）
- 路径：构建时将 `/home/zinn` 替换为 `/root`

## 适用场景

| 适合 | 不适合 |
|------|--------|
| Gazebo 仿真、导航调参、联调主控 | 实车底盘、树莓派原生部署 |
| 团队统一 Melodic 环境 | Paddle OCR（需在宿主机/实机 conda 跑） |

Paddle HTTP 服务若 `--net=host`，容器内主控可访问宿主机 `127.0.0.1:8765`（见 [`control_ws/README.md`](../control_ws/README.md)）。

## 与仿真的关系

容器内启动仿真 → [`nav_sim_ws/QUICKSTART.md`](../nav_sim_ws/QUICKSTART.md) 中的 launch（如 `nav_sim_amcl.launch`）。实机导航 **不要** 在容器里跑，用 [`nav_real_ws`](../nav_real_ws/)。
