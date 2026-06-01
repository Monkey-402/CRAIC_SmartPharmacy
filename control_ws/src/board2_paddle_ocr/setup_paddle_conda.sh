#!/usr/bin/env bash
# 一键安装 Miniconda + paddleocr 环境（装到用户目录，不 conda init，不影响他人默认 shell）
#
# Ubuntu 18.04 / Melodic Docker（glibc 2.27）自动使用旧版 Miniconda 安装包。
# 宿主机 glibc>=2.28 可用最新版；也可强制: MINICONDA_USE_LEGACY=1
set -euo pipefail

INSTALL_DIR="${MINICONDA_DIR:-$HOME/miniconda3}"
ENV_NAME="${CONDA_ENV_NAME:-paddleocr}"
PYTHON_VER="${CONDA_PYTHON_VERSION:-3.9}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REQ_FILE="${SCRIPT_DIR}/paddle_ocr/requirements-paddle.txt"

echo "==> Miniconda 目录: ${INSTALL_DIR}"
echo "==> Conda 环境名:   ${ENV_NAME} (Python ${PYTHON_VER})"

_glibc_version() {
  ldd --version 2>/dev/null | awk '{print $NF; exit}'
}

_need_legacy_installer() {
  if [[ -n "${MINICONDA_INSTALLER:-}" ]]; then
    return 1
  fi
  if [[ "${MINICONDA_USE_LEGACY:-}" == "1" ]]; then
    return 0
  fi
  if [[ "${MINICONDA_USE_LEGACY:-}" == "0" ]]; then
    return 1
  fi
  local ver major minor
  ver="$(_glibc_version)"
  if [[ -z "${ver}" ]]; then
    return 0
  fi
  major="${ver%%.*}"
  minor="${ver#*.}"
  minor="${minor%%.*}"
  if [[ "${major}" -lt 2 ]]; then
    return 0
  fi
  if [[ "${major}" -eq 2 && "${minor}" -lt 28 ]]; then
    return 0
  fi
  return 1
}

_pick_miniconda_sh() {
  local arch="$1"
  local legacy="$2"

  if [[ -n "${MINICONDA_INSTALLER:-}" ]]; then
    echo "${MINICONDA_INSTALLER}"
    return
  fi

  if [[ "${legacy}" == "1" ]]; then
    case "${arch}" in
      x86_64)  echo "Miniconda3-py39_4.12.0-Linux-x86_64.sh" ;;
      aarch64) echo "Miniconda3-py39_4.12.0-Linux-aarch64.sh" ;;
      *) echo "unsupported" ;;
    esac
  else
    case "${arch}" in
      x86_64)  echo "Miniconda3-latest-Linux-x86_64.sh" ;;
      aarch64) echo "Miniconda3-latest-Linux-aarch64.sh" ;;
      *) echo "unsupported" ;;
    esac
  fi
}

_download() {
  local url="$1"
  local dest="$2"
  if command -v curl >/dev/null 2>&1; then
    curl -fsSL -o "${dest}" "${url}"
  elif command -v wget >/dev/null 2>&1; then
    wget -q -O "${dest}" "${url}"
  else
    echo "需要 curl 或 wget。Docker 内可: apt-get update && apt-get install -y curl" >&2
    exit 1
  fi
}

install_miniconda() {
  if [[ -x "${INSTALL_DIR}/bin/conda" ]]; then
    echo "==> 已存在 Miniconda，跳过下载"
    return 0
  fi

  local arch legacy miniconda_sh url glibc_ver
  arch="$(uname -m)"
  glibc_ver="$(_glibc_version)"
  if _need_legacy_installer; then
    legacy=1
    echo "==> 检测到 glibc ${glibc_ver:-未知}，使用旧版 Miniconda（兼容 2.27 / Ubuntu 18.04）"
  else
    legacy=0
    echo "==> glibc ${glibc_ver:-未知}，使用最新 Miniconda 安装包"
  fi

  miniconda_sh="$(_pick_miniconda_sh "${arch}" "${legacy}")"
  if [[ "${miniconda_sh}" == "unsupported" ]]; then
    echo "不支持的架构: ${arch}" >&2
    exit 1
  fi

  TMP="$(mktemp -d)"
  trap 'rm -rf "${TMP}"' EXIT
  INSTALLER="${TMP}/miniconda.sh"
  URL="https://repo.anaconda.com/miniconda/${miniconda_sh}"

  echo "==> 下载 ${URL}"
  _download "${URL}" "${INSTALLER}"

  echo "==> 安装到 ${INSTALL_DIR}（-b 批处理，不修改 ~/.bashrc）"
  bash "${INSTALLER}" -b -p "${INSTALL_DIR}"
}

source_conda() {
  # shellcheck disable=SC1090
  source "${INSTALL_DIR}/etc/profile.d/conda.sh"
  conda config --set auto_activate_base false 2>/dev/null || true
}

create_env() {
  source_conda
  if conda env list | awk '{print $1}' | grep -qx "${ENV_NAME}"; then
    echo "==> 环境 ${ENV_NAME} 已存在，跳过 create"
  else
    echo "==> 创建环境 ${ENV_NAME}"
    conda create -n "${ENV_NAME}" "python=${PYTHON_VER}" -y
  fi
  conda activate "${ENV_NAME}"
  # shellcheck disable=SC1090
  source "${SCRIPT_DIR}/paddle_ocr/bootstrap_pip_py39.sh"
  bootstrap_pip_py39
  echo "==> 安装依赖（CPU 版 Paddle，可能较慢）"
  pip install -r "${REQ_FILE}"
}

write_activate_hint() {
  ACTIVATE_SNIPPET="${SCRIPT_DIR}/activate_paddle_env.sh"
  cat > "${ACTIVATE_SNIPPET}" <<EOF
#!/usr/bin/env bash
# 由 setup_paddle_conda.sh 生成：仅在本终端启用 paddleocr 环境
source "${INSTALL_DIR}/etc/profile.d/conda.sh"
conda activate ${ENV_NAME}
export PADDLE_OCR_HOST="\${PADDLE_OCR_HOST:-127.0.0.1}"
export PADDLE_OCR_PORT="\${PADDLE_OCR_PORT:-8765}"
export PADDLE_OCR_URL="http://\${PADDLE_OCR_HOST}:\${PADDLE_OCR_PORT}"
echo "conda env: ${ENV_NAME}"
echo "PADDLE_OCR_URL=\${PADDLE_OCR_URL}"
EOF
  chmod +x "${ACTIVATE_SNIPPET}"
  echo ""
  echo "=============================================="
  echo "安装完成。"
  echo "  启用环境:  source ${ACTIVATE_SNIPPET}"
  echo "  启动服务:  ${SCRIPT_DIR}/run_paddle_ocr_server.sh"
  echo "  健康检查:  rosrun board2_paddle_ocr paddle_ocr_client.py --health"
  echo "  离线识别:  rosrun board2_paddle_ocr decode_board2_paddle.py /path/to.jpg"
  echo "  ROS 启动:  roslaunch move_nav control.launch use_paddle_ocr:=true"
  echo "=============================================="
  echo "未执行 conda init，不会影响其他同学的默认 shell。"
}

install_miniconda
create_env
write_activate_hint
