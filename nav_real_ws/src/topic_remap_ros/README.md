# topic_remap_ros

在**同一 ROS master**下，把实车 `robot_ws` 发布的话题名转发为 `craic` 导航/控制节点常用名。

## 默认映射

| 输入（实车常见） | 输出（craic 常见） |
|------------------|-------------------|
| `/camera/rgb/image_raw` | `/camera/image_raw` |

激光 scan **不再经本包转发**：`nav_real_ws` 的 Hector / AMCL / costmap 已直接订阅 `/scan_filtered`（`frame_id: base_laser_link`）。  
若需恢复 scan 转发，在 launch 里设 `relay_scan:=true`（不推荐，易与 lslidar 的 `/scan` 冲突）。

## 使用

```bash
roslaunch topic_remap_ros topic_remap_default.launch
```

参数可在 launch 里改，或加载同名私有参数覆盖 `rgb_in` / `scan_in` / `scan_frame_id` 等。

## 与 `nav_real` 联调顺序建议

1. 实车启动底盘与传感器（含 `/scan` → `laser_filter` → `/scan_filtered`、`odom`、`TF`）
2. `roslaunch car_sim nav_real_amcl.launch` 或 `nav_real_hector.launch`
3. 若需相机话题对齐：`nav_real_*_with_remap.launch`（仅转发相机，不碰 scan）

`*_with_remap.launch` 内含 `topic_remap_default.launch`；默认 `relay_scan=false`。
