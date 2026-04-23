# CRAIC

本仓库包含两个主要 ROS 工作空间，分别面向仿真导航与任务控制开发。

## 项目总览

- `nav_sim_ws`：导航仿真工作空间
  - 药房场景 Gazebo 仿真（`yaofang`）
  - 简化小车模型（`car_simple`）
  - `AMCL + move_base + TEB` 导航链路
  - 详细说明见：`nav_sim_ws/README.md`
  - 快速启动见：`nav_sim_ws/QUICKSTART.md`

- `control_ws`：任务控制与业务编排工作空间
  - 导航任务编排与视觉任务分发
  - 支持人物识别、车牌识别等服务式接入
  - 当前推荐开发入口：`control_ws/src/move_nav/src/control_node.cpp`
  - 详细说明见：`control_ws/README.md`

## 目录结构

```text
craic/
├── nav_sim_ws/      # 仿真与导航参数调试
├── control_ws/      # 控制逻辑与任务编排
└── image.png        # 当前仓库内使用的示例贴图资源
```

## 推荐开发流程

1. 在 `nav_sim_ws` 中完成场景、模型、导航参数调通
2. 在 `control_ws` 中对接任务控制逻辑与视觉能力
3. 两侧通过统一目标点、话题约定和任务协议做联调

## 文档入口

- 仿真导航文档：`nav_sim_ws/README.md`
- 仿真快速启动：`nav_sim_ws/QUICKSTART.md`
- 控制工作空间文档：`control_ws/README.md`
