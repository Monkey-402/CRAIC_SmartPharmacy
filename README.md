# CRAIC

智慧药房赛项 ROS 工作空间集合：**仿真导航**、**实机导航**、**任务控制**。

**从这里开始 → [`QUICKSTART.md`](QUICKSTART.md)**

## 工作空间

| 目录 | 说明 |
|------|------|
| [`nav_sim_ws`](nav_sim_ws/) | Gazebo 仿真，`AMCL + move_base + TEB` |
| [`nav_real_ws`](nav_real_ws/) | 实机导航（默认不启 Gazebo） |
| [`control_ws`](control_ws/) | 药房主控、二维码/OCR、裁判上报 |
| [`robot_ws`](robot_ws/) | 实车底盘 launch 模板（1 号车 `192.168.124.3`，2 号车 `192.168.124.9`） |

## 推荐流程

1. `nav_sim_ws` 调通导航与参数  
2. `nav_real_ws` + 实车底盘联调  
3. `control_ws` 跑完整药房任务  

## 文档索引

| 文档 | 用途 |
|------|------|
| **[QUICKSTART.md](QUICKSTART.md)** | **总快速上手（首选）** |
| [nav_sim_ws/QUICKSTART.md](nav_sim_ws/QUICKSTART.md) | 仿真、Docker 细节 |
| [nav_real_ws/QUICKSTART.md](nav_real_ws/QUICKSTART.md) | 实车验收与排错 |
| [control_ws/README.md](control_ws/README.md) | 主控、视觉、裁判 TCP |
| [GIT_WORKFLOW.md](GIT_WORKFLOW.md) | Git 提交规范 |
