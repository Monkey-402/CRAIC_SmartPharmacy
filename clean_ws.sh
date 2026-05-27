#!/usr/bin/env bash
# 清理 craic 下所有 catkin 工作空间的 build / devel 目录。
#
# 用法:
#   ./clean_ws.sh          # 交互确认
#   ./clean_ws.sh -y       # 跳过确认

set -euo pipefail

CRAIC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACES=(nav_sim_ws nav_real_ws control_ws robot_ws)

confirm=true
if [[ "${1:-}" == "-y" || "${1:-}" == "--yes" ]]; then
  confirm=false
fi

removed=0
for ws in "${WORKSPACES[@]}"; do
  ws_dir="${CRAIC_ROOT}/${ws}"
  [[ -d "${ws_dir}" ]] || continue

  for dir in build devel; do
    target="${ws_dir}/${dir}"
    if [[ -e "${target}" ]]; then
      if ${confirm}; then
        read -r -p "删除 ${target}? [y/N] " ans
        [[ "${ans}" == "y" || "${ans}" == "Y" ]] || continue
      fi
      rm -rf "${target}"
      echo "已删除: ${target}"
      removed=$((removed + 1))
    fi
  done

  # catkin_make 生成的 src/CMakeLists.txt 符号链接（常指向容器内 ROS 路径，宿主机失效）
  catkin_link="${ws_dir}/src/CMakeLists.txt"
  if [[ -L "${catkin_link}" ]]; then
    if ${confirm}; then
      read -r -p "删除 ${catkin_link}? [y/N] " ans
      [[ "${ans}" == "y" || "${ans}" == "Y" ]] || continue
    fi
    rm -f "${catkin_link}"
    echo "已删除: ${catkin_link}"
    removed=$((removed + 1))
  fi
done

if [[ ${removed} -eq 0 ]]; then
  echo "没有找到需要清理的 build/devel 目录。"
else
  echo "完成，共清理 ${removed} 个目录。"
fi
