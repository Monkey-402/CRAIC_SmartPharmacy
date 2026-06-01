#!/usr/bin/env bash
# 在 conda 环境 paddleocr 中启动 PaddleOCR HTTP 服务
set -euo pipefail

_resolve_pkg_dir() {
  if command -v rospack >/dev/null 2>&1; then
    local p
    p="$(rospack find board2_paddle_ocr 2>/dev/null || true)"
    if [[ -n "${p}" ]]; then
      echo "${p}"
      return 0
    fi
  fi
  cd "$(dirname "${BASH_SOURCE[0]}")" && pwd
}

PKG_DIR="$(_resolve_pkg_dir)"
INSTALL_DIR="${MINICONDA_DIR:-$HOME/miniconda3}"
ENV_NAME="${CONDA_ENV_NAME:-paddleocr}"

if [[ ! -f "${INSTALL_DIR}/bin/conda" ]]; then
  echo "未找到 Miniconda，请先运行: ${PKG_DIR}/setup_paddle_conda.sh" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "${INSTALL_DIR}/etc/profile.d/conda.sh"
conda activate "${ENV_NAME}"

export PADDLE_OCR_HOST="${PADDLE_OCR_HOST:-127.0.0.1}"
export PADDLE_OCR_PORT="${PADDLE_OCR_PORT:-8765}"
export PADDLE_OCR_URL="http://${PADDLE_OCR_HOST}:${PADDLE_OCR_PORT}"

SERVER_PY="${PKG_DIR}/paddle_ocr/paddle_ocr_server.py"
if [[ ! -f "${SERVER_PY}" ]]; then
  SERVER_PY="$(rospack find board2_paddle_ocr)/paddle_ocr/paddle_ocr_server.py"
fi

echo "Paddle OCR 服务: ${PADDLE_OCR_URL}"
echo "健康检查: rosrun board2_paddle_ocr paddle_ocr_client.py --health"
exec python "${SERVER_PY}"
