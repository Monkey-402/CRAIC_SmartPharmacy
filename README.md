# CRAIC

智慧药房赛项 ROS 工作空间：**仿真导航** → **实机导航** → **任务控制**。

| 文档 | 用途 |
|------|------|
| **[QUICKSTART.md](QUICKSTART.md)** | 实车全栈快速启动（命令为主） |
| **[TECH_REPORT.md](TECH_REPORT.md)** | 赛项技术报告 |
| **[GIT_WORKFLOW.md](GIT_WORKFLOW.md)** | Git 协作规范 |

## 文档约定

每个工作空间维护 **README**（架构、原理、参数）与 **QUICKSTART**（编译、launch 命令），二者尽量不重复：说明看 README，动手看 QUICKSTART。

| 层级 | README | QUICKSTART |
|------|--------|------------|
| 仓库根 | 本页：总览与索引 | 实车 A～D 终端命令摘要 |
| `docker/` | Melodic 仿真镜像 | 安装、构建、运行容器 |
| `nav_sim_ws` / `nav_real_ws` / `control_ws` / `robot_ws` | 各工作空间原理 | 各工作空间启动命令 |
| `control_ws` 内视觉子包 | 算法流程与服务接口 | 由 `control_ws/QUICKSTART` 统一拉起 |

## 工作空间

| 目录 | 说明 | 文档 |
|------|------|------|
| [`docker/`](docker/) | Ubuntu 18.04 + Melodic 仿真容器 | [README](docker/README.md) · [QUICKSTART](docker/QUICKSTART.md) |
| [`nav_sim_ws`](nav_sim_ws/) | Gazebo 仿真，`AMCL + move_base + TEB` | [README](nav_sim_ws/README.md) · [QUICKSTART](nav_sim_ws/QUICKSTART.md) |
| [`nav_real_ws`](nav_real_ws/) | 实机导航（默认不启 Gazebo） | [README](nav_real_ws/README.md) · [QUICKSTART](nav_real_ws/QUICKSTART.md) |
| [`control_ws`](control_ws/) | 药房主控、二维码 / 板二 OCR、裁判与双车 TCP | [README](control_ws/README.md) · [QUICKSTART](control_ws/QUICKSTART.md) |
| [`robot_ws`](robot_ws/) | 实车底盘 launch 模板（拷到小车 `~/robot_ws`） | [README](robot_ws/README.md) · [QUICKSTART](robot_ws/QUICKSTART.md) |

**实车 IP**：1 号 `192.168.124.3`，2 号 `192.168.124.9`。

## 推荐流程

1. `nav_sim_ws` 调通导航与 TEB 参数  
2. `nav_real_ws` + `robot_ws` 实车联调  
3. `control_ws` 跑完整药房任务  

## 实车前提（摘要）

- 小车已装官方 `eprobot_start`、雷达驱动等；本仓库 `robot_ws` 已同步到小车并 `catkin_make`。  
- 终端 A 默认 `pub_odom_tf:=false`（EKF 发 odom TF）；**不用 EKF** 时 A 改为 `pub_odom_tf:=true`，导航用 `nav_real_amcl_no_ekf*.launch`。  
- 视觉依赖（Melodic / Python 2）、板二 Paddle 可选方案：见 [`control_ws/README.md`](control_ws/README.md)。

## 其它参考

| 资源 | 说明 |
|------|------|
| [`judgement.md`](../judgement.md) | 裁判 JSON 协议 |
| [`lh.txt`](lh.txt) | 任务点手动 Nav Goal 坐标 |
| [`nav_sim_ws/photo/README.md`](nav_sim_ws/photo/README.md) | 仿真识别板贴图同步 |
| [`docker/README.md`](docker/README.md) | Melodic 仿真容器 |
