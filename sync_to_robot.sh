#!/usr/bin/env bash
# 同步 craic 到实车，排除 .git / build / devel 及 catkin 临时符号链接。
#
# 用法:
#   ./sync_to_robot.sh                    # 默认 EPRobot@192.168.124.3
#   ./sync_to_robot.sh user@192.168.1.10
#
# 若小车 ~/craic/.git 权限报错，先在小车上执行:
#   rm -rf ~/craic
# 或:
#   sudo chown -R $USER:$USER ~/craic

set -euo pipefail

CRAIC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT="${1:-EPRobot@192.168.124.3}"
DEST="${ROBOT}:~/craic/"

echo "同步 ${CRAIC_ROOT} -> ${DEST}"
echo "排除: .git  build/  devel/  install/  src/CMakeLists.txt"

rsync -avz --delete \
  --exclude='.git/' \
  --exclude='build/' \
  --exclude='devel/' \
  --exclude='install/' \
  --exclude='src/CMakeLists.txt' \
  "${CRAIC_ROOT}/" "${DEST}"

echo "完成。小车上编译示例:"
echo "  ssh ${ROBOT}"
echo "  cd ~/craic/nav_real_ws && catkin_make && source devel/setup.bash"
