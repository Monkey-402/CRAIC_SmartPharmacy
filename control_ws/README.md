# control_ws 说明（智慧社区示例工程）

这个工作空间主要用于智慧社区比赛任务，包含导航控制与任务分发模块。  
当前约定如下：

- `src/move_nav/src/control_node.cpp`：当前重构后的**抽象控制节点**（推荐作为二次开发入口）。
- 除上面这个文件外，`control_ws` 里的其余内容基本都可视为**智慧社区比赛方案示例**，可参考其功能与组织方式。

---

## 目录概览

- `src/move_nav/`
  - 导航控制包
  - `launch/control.launch`：控制节点启动入口
  - `src/control_node.cpp`：当前抽象版主控（视觉能力通过话题接口接入）
- `world/`
  - 比赛相关地图/世界文件资源

---

## 架构说明（当前推荐）

`move_nav` 中的 `control_node.cpp` 目前已改为“导航编排 + 任务分发”模式：

1. 到达指定导航点后抓图保存
2. 发布抽象任务请求（而不是直接调用某个视觉服务）
3. 订阅任务结果回传

默认话题：

- 请求：`smartcommunity/task_request`（`std_msgs/String`）
- 回传：`smartcommunity/task_result`（`std_msgs/String`）

请求消息内容示例（字符串键值对）：

`task_type=people_detection;goal_index=2;image_path=/home/xxx/snapshots/0.jpg;timestamp=1712345678.12`

你可以把任意视觉算法节点接到这个接口上，只要订阅请求并发布结果即可。

---

## 快速使用

### 1) 编译

在 `control_ws` 工作空间根目录执行：

```bash
catkin_make
source devel/setup.bash
```

### 2) 启动控制节点

```bash
roslaunch move_nav control.launch
```

---

## 二次开发建议

- 以 `control_node.cpp` 为主，视觉能力通过独立节点接入 `smartcommunity/task_request` / `task_result`。
- 建议后续把 `task_request/task_result` 从 `std_msgs/String` 升级为自定义消息（字段更清晰、可扩展）。

---

## 备注

在正式比赛或部署前，建议统一检查：

- 抓图保存目录与权限（如 `/root/snapshots`）
- 摄像头话题名（如 `/camera/image_raw`）
- 地图/world 与导航参数匹配
