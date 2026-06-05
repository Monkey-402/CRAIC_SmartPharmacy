# board2_paddle_ocr

识别板二 OCR：ROS Melodic（Python 2.7）通过 HTTP 调用本机 PaddleOCR 服务，接口与 `text_recognition` 相同（`move_nav/Board2Decode`）。

## 架构

| 组件 | 环境 | 说明 |
|------|------|------|
| `ocr_paddle_service.py` | Py2 / ROS | 注册 `board2_decode` 服务，转发 HTTP |
| `paddle_ocr/paddle_ocr_server.py` | Py3 / conda | Flask `POST /ocr/board2`，整图 PaddleOCR |

Docker 使用 `--net=host` 时，容器内 `http://127.0.0.1:8765` 可访问宿主机上的 Paddle 服务。

## 一次性安装（宿主机或 Docker 内）

默认使用 **Miniforge**（`conda-forge`），树莓派 4 / aarch64 比官方 Miniconda 更不易出现 `Illegal instruction`。安装路径仍为 `~/miniconda3`（与旧脚本兼容）。

```bash
roscd board2_paddle_ocr
chmod +x setup_paddle_conda.sh run_paddle_ocr_server.sh
./setup_paddle_conda.sh
```

**树莓派 / 国内网络慢：**

```bash
CONDA_MIRROR=tsinghua ./setup_paddle_conda.sh
```

**已下载安装包到本机：**

```bash
CONDA_INSTALLER_LOCAL=~/Miniforge3-Linux-aarch64.sh ./setup_paddle_conda.sh
```

仍要用官方 Miniconda（不推荐 ARM）：`CONDA_DISTRO=miniconda ./setup_paddle_conda.sh`

## 启动 Paddle HTTP 服务

```bash
roscd board2_paddle_ocr
./run_paddle_ocr_server.sh
```

另开终端健康检查：

```bash
rosrun board2_paddle_ocr paddle_ocr_client.py --health
```

## ROS 集成

默认 Tesseract（`text_recognition`）。启用 Paddle 与实车 launch → [`../../QUICKSTART.md`](../../QUICKSTART.md) §6。

```bash
roslaunch move_nav control.launch use_paddle_ocr:=true
```

可选参数：`paddle_ocr_url:=http://127.0.0.1:8765`、`paddle_ocr_timeout:=120.0`

## 离线测图

```bash
rosrun board2_paddle_ocr decode_board2_paddle.py /path/to/11.jpg \
  --snapshot-dir $(rospack find move_nav)/../snapshots
```
