#!/usr/bin/env bash
# 一键安装 Conda（默认 Miniforge，ARM/树莓派更稳）+ paddleocr 环境
# 装到用户目录，不 conda init，不影响他人默认 shell。
#
# 环境变量：
#   CONDA_DIR / MINICONDA_DIR  安装路径（默认 ~/miniconda3，与旧脚本兼容）
#   CONDA_DISTRO               miniforge（默认）| miniconda
#   CONDA_MIRROR               tsinghua | official（默认 official）
#   CONDA_INSTALLER            安装包文件名（覆盖自动选择）
#   CONDA_INSTALLER_LOCAL      本地 .sh 路径（跳过下载）
#   CONDA_ENV_NAME             环境名（默认 paddleocr）
#   CONDA_PYTHON_VERSION       Python 版本（默认 3.9）
#   MINICONDA_USE_LEGACY=1     仅 miniconda 时：glibc 2.27 用旧版包
set -euo pipefail

INSTALL_DIR="${CONDA_DIR:-${MINICONDA_DIR:-$HOME/miniconda3}}"
ENV_NAME="${CONDA_ENV_NAME:-paddleocr}"
PYTHON_VER="${CONDA_PYTHON_VERSION:-3.9}"
CONDA_DISTRO="${CONDA_DISTRO:-miniforge}"
CONDA_MIRROR="${CONDA_MIRROR:-official}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REQ_FILE="${SCRIPT_DIR}/paddle_ocr/requirements-paddle.txt"

echo "==> Conda 目录:   ${INSTALL_DIR}"
echo "==> 发行版:       ${CONDA_DISTRO}（ARM/aarch64 推荐 miniforge）"
echo "==> 环境名:       ${ENV_NAME} (Python ${PYTHON_VER})"

_glibc_version() {
  ldd --version 2>/dev/null | awk '{print $NF; exit}'
}

_need_legacy_miniconda() {
  if [[ "${CONDA_DISTRO}" != "miniconda" ]]; then
    return 1
  fi
  if [[ -n "${CONDA_INSTALLER:-}" ]]; then
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

_pick_installer_name() {
  local arch="$1"
  local legacy="$2"

  if [[ -n "${CONDA_INSTALLER:-${MINICONDA_INSTALLER:-}}" ]]; then
    echo "${CONDA_INSTALLER:-${MINICONDA_INSTALLER:-}}"
    return
  fi

  if [[ "${CONDA_DISTRO}" == "miniforge" ]]; then
    case "${arch}" in
      x86_64)  echo "Miniforge3-Linux-x86_64.sh" ;;
      aarch64) echo "Miniforge3-Linux-aarch64.sh" ;;
      armv7l|armv6l) echo "Miniforge3-Linux-armv7l.sh" ;;
      *) echo "unsupported" ;;
    esac
    return
  fi

  # miniconda（可选回退）
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

_installer_url() {
  local name="$1"
  case "${CONDA_DISTRO}" in
    miniforge)
      case "${CONDA_MIRROR}" in
        tsinghua|tuna)
          echo "https://mirrors.tuna.tsinghua.edu.cn/github-release/conda-forge/miniforge/LatestRelease/${name}"
          ;;
        *)
          echo "https://github.com/conda-forge/miniforge/releases/latest/download/${name}"
          ;;
      esac
      ;;
    miniconda)
      echo "https://repo.anaconda.com/miniconda/${name}"
      ;;
    *)
      echo "不支持的 CONDA_DISTRO: ${CONDA_DISTRO}" >&2
      return 1
      ;;
  esac
}

_download() {
  local url="$1"
  local dest="$2"
  if command -v curl >/dev/null 2>&1; then
    curl -fsSL -o "${dest}" "${url}"
  elif command -v wget >/dev/null 2>&1; then
    wget -q -O "${dest}" "${url}"
  else
    echo "需要 curl 或 wget。可: apt-get install -y curl" >&2
    exit 1
  fi
}

_verify_conda_binary() {
  if ! "${INSTALL_DIR}/bin/conda" --version >/dev/null 2>&1; then
    echo "错误: ${INSTALL_DIR}/bin/conda 无法运行（如 Illegal instruction）。" >&2
    echo "  树莓派请确认: uname -m 为 aarch64，并 rm -rf ${INSTALL_DIR} 后重试。" >&2
    echo "  国内可先下载安装包再: CONDA_INSTALLER_LOCAL=~/Miniforge3-Linux-aarch64.sh $0" >&2
    echo "  或: CONDA_MIRROR=tsinghua $0" >&2
    return 1
  fi
  return 0
}

install_conda_base() {
  if [[ -x "${INSTALL_DIR}/bin/conda" ]] && _verify_conda_binary; then
    echo "==> 已存在可用 Conda（${INSTALL_DIR}），跳过下载"
    return 0
  fi

  if [[ -d "${INSTALL_DIR}" ]]; then
    echo "==> 清理不完整安装: ${INSTALL_DIR}"
    rm -rf "${INSTALL_DIR}"
  fi

  local arch legacy installer_name url glibc_ver
  arch="$(uname -m)"
  glibc_ver="$(_glibc_version)"

  if _need_legacy_miniconda; then
    legacy=1
    echo "==> glibc ${glibc_ver:-未知}，miniconda 使用旧版安装包"
  else
    legacy=0
    if [[ "${CONDA_DISTRO}" == "miniforge" ]]; then
      echo "==> 使用 Miniforge（conda-forge，适合树莓派 / aarch64）"
      if [[ "${arch}" == "aarch64" && "${CONDA_MIRROR}" == "official" ]]; then
        echo "    国内较慢可: CONDA_MIRROR=tsinghua ./setup_paddle_conda.sh"
      fi
    else
      echo "==> glibc ${glibc_ver:-未知}，使用最新 Miniconda 安装包"
    fi
  fi

  installer_name="$(_pick_installer_name "${arch}" "${legacy}")"
  if [[ "${installer_name}" == "unsupported" ]]; then
    echo "不支持的架构: ${arch}" >&2
    exit 1
  fi

  TMP="$(mktemp -d)"
  trap 'rm -rf "${TMP}"' EXIT
  INSTALLER="${TMP}/conda-installer.sh"

  if [[ -n "${CONDA_INSTALLER_LOCAL:-}" && -f "${CONDA_INSTALLER_LOCAL}" ]]; then
    echo "==> 使用本地安装包: ${CONDA_INSTALLER_LOCAL}"
    cp "${CONDA_INSTALLER_LOCAL}" "${INSTALLER}"
  else
    url="$(_installer_url "${installer_name}")"
    echo "==> 下载 ${url}"
    _download "${url}" "${INSTALLER}"
  fi

  echo "==> 安装到 ${INSTALL_DIR}（-b 批处理，不修改 ~/.bashrc）"
  bash "${INSTALLER}" -b -p "${INSTALL_DIR}"

  echo "==> 验证 conda 可执行"
  _verify_conda_binary
  "${INSTALL_DIR}/bin/conda" --version
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
  echo "==> 安装依赖（CPU 版 Paddle，可能较慢；树莓派可用 CONDA_MIRROR=tsinghua 仅加速安装器下载）"
  if [[ "${CONDA_MIRROR}" == "tsinghua" || "${CONDA_MIRROR}" == "tuna" ]]; then
    pip install -r "${REQ_FILE}" -i https://pypi.tuna.tsinghua.edu.cn/simple
  else
    pip install -r "${REQ_FILE}"
  fi
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

install_conda_base
create_env
write_activate_hint
