# 识别板二 OCR 替代方案

当前默认：**Tesseract + adaptive 二值化 + ROI + 单次 `--psm 7` + 赛规关键词推断**（`use_keyword_inference`）。

识别板二文案高度固定（「化验区空闲/忙碌 + 等待 N 秒」），不必追求通用 OCR 全能。

## 方案对比

| 方案 | 准确度 | Pi 速度 | 与现架构 | 说明 |
|------|--------|---------|----------|------|
| **A. Tesseract + 关键词推断**（现用） | 中 | 快 | 已集成 | 乱码时靠「化/空/忙/秒」拼标准句；**优先调 ROI** |
| **B. 纯关键词 / 模板**（官方 `OCR_CAM_0506.py`） | 高* | 最快 | 易改 `ocr_service.py` | 不做整句 OCR，只统计 ROI 内是否出现关键字；*依赖 ROI 准 |
| **C. PaddleOCR lite** | 高 | 慢 | 需 Python3 或 Docker | 中文屏显最好；Melodic/Py2 实车安装麻烦 |
| **D. EasyOCR** | 中高 | 很慢 | 需 Py3 + 模型 | 不推荐 Pi 实车在线用 |
| **E. 云端 API** | 高 | 看网络 | HTTP 调用 | 比赛现场网络不可控，不推荐 |
| **F. 颜色/亮度判定** | 中 | 极快 | 可并行 | 若屏幕「空闲=绿、忙碌=红」可辅助；赛规未必固定 |

## 推荐路线

1. **短期（不换引擎）**  
   - 用 `*_ocr_roi.jpg` / `*_ocr_bin.jpg` 微调 `roi_*_ratio`  
   - 保持 `use_keyword_inference:=true`  
   - 仍不准时加强方案 **B**：ROI 内分块 + 字模/关键词计数（与官方例程一致）

2. **中期（Docker / 笔记本侧）**  
   - 在 **Docker（Python3）** 里加 **PaddleOCR** 节点，仍暴露同一 `Board2Decode` 服务  
   - 实车 Pi 只跑导航+抓图，OCR 在算力更好的机器上（需同 ROS master 或传图路径）

3. **不推荐**  
   - 实车 Pi + EasyOCR 在线  
   - 整图 Tesseract 不做 ROI  

## 离线测试

```bash
cd ~/craic/control_ws && source devel/setup.bash
rosservice call /yaofang_vision/board2_decode \
  "image_path: '/home/EPRobot/craic/control_ws/src/snapshots/你的图.jpg'"
```

查看日志中的 `raw=` 与最终 `speech_text=`，判断是 OCR 乱码还是关键词推断未命中。
