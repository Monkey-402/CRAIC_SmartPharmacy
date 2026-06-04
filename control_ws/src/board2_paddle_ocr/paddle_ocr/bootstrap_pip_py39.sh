#!/usr/bin/env bash
# 在已激活的 conda 环境中，为 Python 3.9 安装 pip<24
# 用法: source .../bootstrap_pip_py39.sh && bootstrap_pip_py39

bootstrap_pip_py39() {
  local pyver major minor sp
  pyver="$(python -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
  major="${pyver%%.*}"
  minor="${pyver#*.}"

  if [[ "${major}" != "3" ]] || [[ "${minor}" -lt 9 ]] || [[ "${minor}" -ge 10 ]]; then
    echo "bootstrap_pip_py39: 仅针对 Python 3.9，当前 ${pyver}" >&2
    return 1
  fi

  sp="${CONDA_PREFIX}/lib/python${pyver}/site-packages"
  echo "==> 移除损坏的 pip 包: ${sp}/pip*"
  rm -rf "${sp}/pip" "${sp}"/pip-*.dist-info 2>/dev/null || true

  local ver tried=0
  for ver in 23.3.1 23.2.1 22.3.1 22.1.2; do
    echo "==> 尝试 conda install pip=${ver}"
    if conda install -y "pip=${ver}" wheel setuptools 2>/dev/null; then
      tried=1
      if pip --version >/dev/null 2>&1; then
        echo "==> conda 安装 pip 成功: $(pip --version)"
        return 0
      fi
      echo "==> pip=${ver} 仍不可用，继续尝试"
      rm -rf "${sp}/pip" "${sp}"/pip-*.dist-info 2>/dev/null || true
    fi
  done

  echo "==> conda 无可用 pip 包，使用 get-pip.py 安装 pip==23.3.1"
  local tmp get_pip
  tmp="$(mktemp -d)"
  get_pip="${tmp}/get-pip.py"
  if command -v curl >/dev/null 2>&1; then
    curl -fsSL -o "${get_pip}" "https://bootstrap.pypa.io/pip/3.9/get-pip.py"
  elif command -v wget >/dev/null 2>&1; then
    wget -q -O "${get_pip}" "https://bootstrap.pypa.io/pip/3.9/get-pip.py"
  else
    echo "需要 curl 或 wget" >&2
    rm -rf "${tmp}"
    return 1
  fi

  python "${get_pip}" "pip==23.3.1"
  rm -rf "${tmp}"

  pip --version
  echo "==> get-pip 完成"
}
