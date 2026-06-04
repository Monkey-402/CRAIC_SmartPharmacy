#!/usr/bin/env bash
# 同步 craic 到实车，排除 .git / build / devel 及 catkin 临时符号链接；
# 同时将 robot_ws/src/eprobot_chassis_bringup 同步到小车 ~/robot_ws/src/。
#
# 用法:
#   ./sync_to_robot.sh                         # 默认 1 号车 EPRobot@192.168.124.3
#   ./sync_to_robot.sh EPRobot@192.168.124.9  # 2 号车
#
# 若小车 ~/craic/.git 权限报错，先在小车上执行:
#   rm -rf ~/craic
# 或:
#   sudo chown -R $USER:$USER ~/craic

set -euo pipefail

CRAIC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT="${1:-EPRobot@192.168.124.3}"
CRAIC_DEST="${ROBOT}:~/craic/"
CHASSIS_SRC="${CRAIC_ROOT}/robot_ws/src/eprobot_chassis_bringup"
CHASSIS_DEST="${ROBOT}:~/robot_ws/src/eprobot_chassis_bringup"

echo "同步 ${CRAIC_ROOT} -> ${CRAIC_DEST}"
echo "排除: .git  build/  devel/  install/  src/CMakeLists.txt"

rsync -avz --delete \
  --exclude='.git/' \
  --exclude='build/' \
  --exclude='devel/' \
  --exclude='install/' \
  --exclude='src/CMakeLists.txt' \
  "${CRAIC_ROOT}/" "${CRAIC_DEST}"

if [[ ! -d "${CHASSIS_SRC}" ]]; then
  echo "错误: 未找到 ${CHASSIS_SRC}" >&2
  exit 1
fi

echo ""
echo "同步 ${CHASSIS_SRC} -> ${CHASSIS_DEST}"

rsync -avz --delete \
  "${CHASSIS_SRC}/" "${CHASSIS_DEST}/"

echo ""
echo "完成。小车上编译示例:"
echo "  ssh ${ROBOT}"
echo "  cd ~/robot_ws && catkin_make && source devel/setup.bash"
echo "  cd ~/craic/nav_real_ws && catkin_make && source devel/setup.bash"
