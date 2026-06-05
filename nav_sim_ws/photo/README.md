# 识别板贴图（test1 / test2）

## 用法

1. 将新图覆盖本目录下的 **`test1.png`**、**`test2.png`**
   - `test1.png` → Wall_1 识别板（`RecognitionBoard_1`）
   - `test2.png` → Wall_10 识别板（`RecognitionBoard_10`）
2. 运行同步脚本：

```bash
cd ~/CAIR/craic/photo
./sync_test_photos.sh
```

Docker 内路径：`/root/craic/photo/sync_test_photos.sh`

## 脚本会更新两处

| 目标 | 路径 |
|------|------|
| move_nav 测试资源 | `control_ws/src/move_nav/test_assets/` |
| Gazebo 模型贴图 | `nav_sim_ws/src/yaofang_world/models/yaofang/materials/textures/` |

## 建议

- 贴图比例 **16:10**（如 1280×800）与仿真识别板尺寸一致，变形最小。
- 换图后需 **完全重启 Gazebo**；测试发图节点也需重启。
