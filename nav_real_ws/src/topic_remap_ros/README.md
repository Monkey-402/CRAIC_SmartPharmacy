# topic_remap_ros

在**同一 ROS master**下，把实车 `robot_ws` 发布的话题名转发为 `craic` 导航/控制节点常用名。

## 默认映射

| 输入（实车常见） | 输出（craic 常见） |
|------------------|-------------------|
| `/camera/rgb/image_raw` | `/camera/image_raw` |
| `/scan_filtered` | `/scan`（`frame_id` 可改为 `laser_link` 以匹配 `car_sim` 代价地图） |

## 使用

```bash
roslaunch topic_remap_ros topic_remap_default.launch
```

参数可在 launch 里改，或加载同名私有参数覆盖 `rgb_in` / `scan_in` / `scan_frame_id` 等。

## 与 `nav_real` 联调顺序建议

1. 实车启动底盘与传感器（含 `scan`、`odom`、`TF`）
2. `roslaunch topic_remap_ros topic_remap_default.launch`
3. `roslaunch car_sim nav_real.launch`

若实车已直接发布 `/scan` 且 `frame_id` 已是 `laser_link`，可将 `relay_scan` 设为 `false`。
