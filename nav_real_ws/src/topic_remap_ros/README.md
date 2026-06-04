# topic_remap_ros

可选话题转发（**默认全部关闭**）。`control_ws` 与仿真 Gazebo 已直接使用官方相机名 **`/camera/rgb/image_raw`**，实车 `chassis.launch` 亦同，一般**无需**再启本包。

## 历史映射（仅 legacy）

| 输入 | 输出 | 启用参数 |
|------|------|----------|
| `/camera/rgb/image_raw` | `/camera/image_raw` | `relay_rgb:=true` |
| `/scan_filtered` | `/scan`（并改 frame_id） | `relay_scan:=true` |

## 单独启动

```bash
# 默认不转发任何话题（节点会提示后退出）
roslaunch topic_remap_ros topic_remap_default.launch

# 若仍有旧节点订 /camera/image_raw：
roslaunch topic_remap_ros topic_remap_default.launch relay_rgb:=true
```

## 与 nav launch 的关系

`nav_real_*_with_remap.launch` 仍 include 本 launch，但默认 `relay_rgb=false`、`relay_scan=false`，与 `nav_real_*.launch` 行为等价。
