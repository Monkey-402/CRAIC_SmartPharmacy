#!/usr/bin/env bash
# 同步单张识别板贴图到 control_ws + Gazebo 模型两处
# 用法: ./sync_test_photos.sh <图片文件名> <test1|test2>
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRAIC_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DEST_MOVE_NAV="${CRAIC_ROOT}/control_ws/src/move_nav/test_assets"
DEST_GAZEBO="${CRAIC_ROOT}/nav_sim_ws/src/yaofang_world/models/yaofang/materials/textures"

die() {
  echo "错误: $*" >&2
  exit 1
}

usage() {
  echo "用法: $(basename "$0") <图片文件名> <test1|test2>" >&2
  echo "示例: $(basename "$0") my_board.png test1" >&2
  exit 1
}

[[ $# -eq 2 ]] || usage

IMAGE_ARG="$1"
SLOT="$2"

case "$SLOT" in
  test1|test2) ;;
  *) die "第二个参数必须是 test1 或 test2，当前为: ${SLOT}" ;;
esac

if [[ -f "$IMAGE_ARG" ]]; then
  SRC="$IMAGE_ARG"
elif [[ -f "${SCRIPT_DIR}/${IMAGE_ARG}" ]]; then
  SRC="${SCRIPT_DIR}/${IMAGE_ARG}"
else
  die "找不到图片: ${IMAGE_ARG}（可放在 photo/ 目录下或写绝对路径）"
fi

DEST_NAME="${SLOT}.png"

mkdir -p "$DEST_MOVE_NAV" "$DEST_GAZEBO"

install -m 0644 "$SRC" "${DEST_MOVE_NAV}/${DEST_NAME}"
install -m 0644 "$SRC" "${DEST_GAZEBO}/${DEST_NAME}"

echo "已同步识别板贴图:"
echo "  源文件: ${SRC}"
echo "  目标名: ${DEST_NAME}"
echo "  -> ${DEST_MOVE_NAV}/${DEST_NAME}"
echo "  -> ${DEST_GAZEBO}/${DEST_NAME}"
echo ""
echo "请重启 Gazebo（roslaunch）后查看仿真贴图；若在用 test_image_publisher，也请重启该节点。"
