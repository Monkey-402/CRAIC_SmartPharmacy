# AGENTS.md

## 项目定位

这是一个面向“中国机器人及人工智能大赛 - 智慧药房”任务的 ROS1 工程，当前仓库实际由 4 个工作空间组成：

- `nav_sim_ws`：Gazebo 仿真、地图、导航参数调试。
- `nav_real_ws`：实机导航栈，默认不启 Gazebo。
- `control_ws`：任务编排、视觉服务示例、药房/社区控制节点。
- `robot_ws`：实车底盘启动模板。

根目录 `智慧药房线上规则.pdf` 是需求基准，后续任何控制逻辑都应优先对齐该规则，而不是沿用“智慧社区”示例代码的默认假设。

## 线上规则摘要

从规则文件提炼出的实现约束：

- 任务顺序必须是：识别板一 -> 体检区取样 -> 识别板二 -> 化验区送样。
- 识别板一负责给出样本来源窗口 `A/B/C`、样本类型、目标化验窗口 `1/2/3/4`。
- 识别板二负责给出“空闲快速通过”或“忙碌等待 n 秒后通过”，且需要语音播报。
- 取样和送样时，整车需进入方框并明显停留，建议 `1~2s`。
- 支持一次性配送一个二维码中的全部样本，这是加分项。
- 需要向裁判系统实时上报：速度、里程计、视觉识别结果、当前位置、当前任务。
- 允许双车协同，但当前仓库没有实现双车调度。

## 当前工程真实状态

需要先记住这几个事实：

- 默认启动的 `control_ws/src/move_nav/launch/control.launch` 运行的是 `control_node.cpp`，它是“智慧社区”抽象控制示例，不是完整智慧药房主流程。
- 最接近智慧药房规则的代码在：
  - `control_ws/src/move_nav/src/control_node_yaofang_template.cpp`
  - `control_ws/src/move_nav/src/control_node_yaofang_msg_template.cpp`
- 其中字符串版模板 `control_node_yaofang_template.cpp` 目前没有加入 `CMakeLists.txt` 编译。
- 消息版模板 `control_node_yaofang_msg_template.cpp` 已加入编译目标 `yaofang_control_msg_node`，但它依赖的 `TaskResult.msg` 字段定义和源码不一致，当前大概率无法通过编译。
- 当前环境里没有 `catkin_make`，所以本次未能做真实编译验证，只完成了静态代码审阅。

## 功能包总览

### `control_ws/src/move_nav`

作用：

- 任务编排核心包。
- 提供 `control_node.cpp` 抽象导航/拍照/任务分发示例。
- 提供药房模板控制节点源码。
- 定义 `SampleOrder.msg`、`TaskRequest.msg`、`TaskResult.msg`。

关键入口：

- `launch/control.launch`
- `launch/yaofang_msg_mock.launch`
- `src/control_node.cpp`
- `src/control_node_yaofang_template.cpp`
- `src/control_node_yaofang_msg_template.cpp`

当前问题：

- `control.launch` 仍然指向“智慧社区”流程，不是药房比赛默认入口。
- `control_node_yaofang_template.cpp` 没有编译目标，无法直接运行。
- `control_node_yaofang_msg_template.cpp` 使用了 `TaskResult.msg` 中不存在的字段：`has_a`、`has_b`、`has_c`、`delivery_slot`、`sample_count`。
- `control_node_yaofang_template.cpp` 里的 `markPickedAtGoal` 和 `markDeliveredAtGoal` 传参是值拷贝，不会把 `picked/delivered` 状态写回，导致任务流程逻辑上会失败。
- 多处使用硬编码路径 `"/home/zinn/snapshots/"`。
- 仍未实现裁判系统局域网状态上报。
- 语音播报只有占位发布或日志，没有真实 TTS/播放器闭环。

建议改进：

- 统一确定“药房正式主控”只保留一个入口，建议保留消息版。
- 先修正 `TaskRequest/TaskResult` 消息定义，再让控制节点、视觉节点、语音节点共用同一协议。
- 把样本订单、识别板二结果、裁判上报都改成结构化消息，不再用 `std_msgs/String` 拼串。
- 将导航点、等待时长、图片目录改为 ROS 参数。
- 为“取样停留/送样停留/播报/裁判上报”补明确状态机。

### `control_ws/src/yolo_onnx_ros`

作用：

- 提供 ONNX YOLO 检测服务示例。
- 当前更像“人物检测示例服务”，来自智慧社区场景。

关键入口：

- `scripts/yolo_onnx_service_node.py`
- `srv/ObjectDetection.srv`
- `launch/yolo_camera.launch`

当前问题：

- 服务名注册为 `~/detect_objects`，实际会展开成私有服务名，和客户端里常写的 `detect_objects` 容易不一致。
- 类别映射还是 `peopleInside/peopleOutside/object3...`，和智慧药房识别板一/二任务并不匹配。
- `input_topic` 参数在服务端未真正使用。
- `CMakeLists.txt` 里有重复 `catkin_package()`，导出配置不干净。
- 没有安装脚本配置，部署一致性弱。
- 保存目录也写死为 `/home/zinn/snapshots`。

建议改进：

- 改成明确的公共服务名或直接切换为订阅 `TaskRequest` 的任务节点。
- 把它从“人物检测示例”重构为“药房视觉任务适配层”，按 `task_type` 分发 `board1_decode` / `board2_decode`。
- 增加安装规则、参数校验和模型不存在时的错误提示。

### `control_ws/src/license_plate_ocr`

作用：

- 车牌检测 + OCR 服务示例。
- 仍然是智慧社区比赛遗留能力，不属于药房核心需求。

关键入口：

- `scripts/license_plate_server.py`
- `srv/LicensePlateOCR.srv`
- `launch/ocr_server.launch`

当前问题：

- 与智慧药房规则关系弱，默认启动会分散主链路注意力。
- 依旧存在硬编码保存目录。
- `test_ocr_client.py` 使用了外部绝对路径示例。
- 引入 `PaddleOCR` 和 ONNX 运行时，但仓库没有统一依赖说明。

建议改进：

- 如果药房项目不需要车牌能力，建议从默认启动链路移除。
- 若保留，应将其明确标注为“历史示例包”。

### `nav_sim_ws/src/car_sim`

作用：

- 仿真导航主包。
- 管理 Gazebo 启动、地图、AMCL、move_base、RViz、导航参数。

关键入口：

- `launch/nav_sim.launch`
- `launch/car_urdf.launch`
- `launch/move_base.launch`
- `map/map_sim.yaml`
- `param/*.yaml`

当前问题：

- `map/map_sim.yaml` 里的图片路径是绝对路径 `/home/zinn/craic/...`，换机器即失效。
- 包名叫 `car_sim`，但在 `nav_real_ws` 里也沿用同名，语义有些混乱。
- 目前默认局部规划器是 TEB，自定义 `my_local_planner` 没接入默认链路。

建议改进：

- 先把地图路径改成相对路径或用 `$(find car_sim)` 生成。
- 把导航参数按“仿真/实机/比赛”分组。
- 在 README 中补充药房场地点位标定流程。

### `nav_sim_ws/src/robot_description/car_simple`

作用：

- 简化车体 URDF，供 Gazebo 和导航栈使用。

当前问题：

- 仍是简化模型，未体现比赛要求中的阿克曼结构约束。
- 若后续实机使用阿克曼底盘，这个模型与控制约束可能不一致。

建议改进：

- 明确这是“仿真调参模型”还是“比赛一致性模型”。
- 若追求规则一致性，需要补阿克曼运动学与更真实的 footprint。

### `nav_sim_ws/src/yaofang_world`

作用：

- 智慧药房 Gazebo 世界、模型、贴图资源。

当前问题：

- 世界资源是比赛空间基础，但没有配套文档说明各区与规则中的 `A/B/C`、`1/2/3/4`、识别板一/二的坐标对应关系。

建议改进：

- 补一份“场地语义坐标表”，把导航点和比赛区块对应起来。

### `nav_sim_ws/src/my_local_planner`

作用：

- 自定义局部规划器实验包。

当前问题：

- 默认 launch 没有使用它。
- 算法输出 `linear.y`，更像全向底盘控制，不符合规则里的阿克曼底盘约束。
- `if(abs(getDistance(goal_pose,robot_pose)<=0.1))` 这个条件写法有明显错误，`abs()` 作用在布尔表达式上。
- 代码更像实验性质，没有测试，也缺乏碰撞约束说明。

建议改进：

- 若比赛以阿克曼底盘为主，优先继续使用 TEB/DWA 并调好参数。
- 若要保留自研规划器，应按阿克曼约束重写速度模型，并补仿真验证。

### `nav_real_ws/src/car_sim`

作用：

- 实机导航主包。
- 启动 `map_server + amcl + move_base + rviz`，默认不启 Gazebo。

关键入口：

- `launch/nav_real.launch`
- `launch/nav_real_with_remap.launch`

当前问题：

- 继承了仿真包名 `car_sim`，对新接手的人不够直观。
- 与仿真工作空间内容高度重复，后续很容易双份维护漂移。

建议改进：

- 将仿真与实机共享内容抽公共包，减少复制。
- 至少建立同步策略，避免两个工作空间参数长期分叉。

### `nav_real_ws/src/topic_remap_ros`

作用：

- 把实车话题映射成控制栈常用命名。
- 当前只桥接图像和激光。

当前问题：

- 只做了 `/camera/rgb/image_raw -> /camera/image_raw` 和 `/scan_filtered -> /scan`。
- 没处理 odom、tf、IMU、裁判上报等更完整的实机接入问题。

建议改进：

- 视实车实际话题，补 `odom`、相机 `frame_id`、必要静态 TF 和健康检查日志。
- 将 remap 配置参数化，按车型拆配置文件。

### `nav_real_ws/src/robot_description/car_simple`

作用：

- 实机工作空间里保留的同名简化车体描述。

建议改进：

- 若实机不需要 Gazebo 模型，可考虑只保留必要 TF/描述，减少歧义。

### `nav_real_ws/src/yaofang_world`

作用：

- 从仿真复制来的世界资源，默认实机导航不使用。

建议改进：

- 如果只是历史保留，应在文档里明确“非实机默认依赖”。

### `nav_real_ws/src/my_local_planner`

作用：

- 与仿真工作空间相同的自定义局部规划器副本。

建议改进：

- 与 `nav_sim_ws/src/my_local_planner` 合并维护，避免两份相同代码漂移。

### `robot_ws/src/eprobot_chassis_bringup`

作用：

- 实车底盘与雷达启动模板。
- 作为外部 `eprobot_start` 等包的轻量入口。

关键入口：

- `launch/chassis.launch`

当前问题：

- 只是模板，不包含导航、不包含裁判系统上报、不包含健康检查。
- 对外部依赖较强，但根仓库没有集中依赖清单。

建议改进：

- 增加“实车最小依赖检查清单”。
- 补充与 `nav_real_ws` 的联合启动脚本或操作手册。

## 当前最高优先级问题

如果后续要把仓库真正变成“智慧药房可运行版本”，优先顺序建议是：

1. 修正 `move_nav` 的消息定义与药房主控代码不一致的问题。
2. 决定唯一正式主控入口，移除智慧社区旧流程对默认启动的干扰。
3. 完成识别板一、识别板二的视觉任务节点，而不是继续复用人物/车牌示例。
4. 去掉所有绝对路径和本机用户名依赖。
5. 增加裁判系统实时上报模块。
6. 建立真实可运行的语音播报链路。
7. 核对导航模型是否满足阿克曼/比赛尺寸约束。

## 建议的后续重构方向

- `control_ws`：聚焦药房业务，智慧社区历史代码迁到 `legacy/` 或明确标注。
- `vision`：按 `board1_decode`、`board2_decode`、可选 `ocr` 拆分任务，而不是按旧比赛能力拆。
- `navigation`：把仿真与实机共享参数、地图语义、点位管理统一起来。
- `integration`：新增 `judge_bridge` 包，负责状态上报与日志落盘。

## 给后续代理的提示

- 不要默认认为 `control.launch` 就是药房主流程，它现在不是。
- 不要默认 `smartcommunity` 话题名意味着业务仍是智慧社区；这里很多代码只是历史复用。
- 修改药房主控前，先统一消息协议，再接视觉节点。
- 任何新增路径、地图、模型、输出目录都不要写死成本机绝对路径。
