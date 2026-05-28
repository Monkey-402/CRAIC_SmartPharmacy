#!/usr/bin/env bash
# 一键同步识别板贴图：photo/test1.png、test2.png -> control_ws + Gazebo 模型两处
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRAIC_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

SRC1="${SCRIPT_DIR}/test1.png"
SRC2="${SCRIPT_DIR}/test2.png"

DEST_MOVE_NAV="${CRAIC_ROOT}/control_ws/src/move_nav/test_assets"
DEST_GAZEBO="${CRAIC_ROOT}/nav_sim_ws/src/yaofang_world/models/yaofang/materials/textures"

die() {
  echo "错误: $*" >&2
  exit 1
}

for f in "$SRC1" "$SRC2"; do
  [[ -f "$f" ]] || die "缺少 ${f}，请把新图放到 photo/ 并命名为 test1.png、test2.png"
done

mkdir -p "$DEST_MOVE_NAV" "$DEST_GAZEBO"

install -m 0644 "$SRC1" "${DEST_MOVE_NAV}/test1.png"
install -m 0644 "$SRC2" "${DEST_MOVE_NAV}/test2.png"
install -m 0644 "$SRC1" "${DEST_GAZEBO}/test1.png"
install -m 0644 "$SRC2" "${DEST_GAZEBO}/test2.png"

echo "已同步识别板贴图:"
echo "  源目录: ${SCRIPT_DIR}"
echo "  -> ${DEST_MOVE_NAV}/"
echo "  -> ${DEST_GAZEBO}/"
echo ""
echo "请重启 Gazebo（roslaunch）后查看仿真贴图；若在用 test_image_publisher，也请重启该节点。"
