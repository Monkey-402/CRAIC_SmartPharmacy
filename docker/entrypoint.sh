#!/bin/bash
set -e

export LANG=C.UTF-8
export LC_ALL=C.UTF-8
export PYTHONIOENCODING=utf-8

source /opt/ros/melodic/setup.bash

# Overlay navigation workspaces (nav_real chains nav_sim at build time).
# Do not source robot_ws here: it only underlays melodic and would hide nav packages.
if [[ -f "${CRAIC_HOME}/nav_real_ws/devel/setup.bash" ]]; then
  source "${CRAIC_HOME}/nav_real_ws/devel/setup.bash"
elif [[ -f "${CRAIC_HOME}/nav_sim_ws/devel/setup.bash" ]]; then
  source "${CRAIC_HOME}/nav_sim_ws/devel/setup.bash"
fi

if [[ -f "${CRAIC_HOME}/control_ws/devel/setup.bash" ]]; then
  source "${CRAIC_HOME}/control_ws/devel/setup.bash"
fi

exec "$@"
