# CRAIC — Ubuntu 18.04 + ROS Melodic
# 构建上下文为 craic 根目录（见 docker/README.md）
#
#   cd ~/craic
#   docker build -f docker/Dockerfile -t craic:melodic .
#
# 或使用 compose（推荐）:
#   cd ~/craic/docker && docker compose build

FROM osrf/ros:melodic-desktop-full

ARG CRAIC_HOME=/root/craic

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONIOENCODING=utf-8 \
    ROS_DISTRO=melodic \
    CRAIC_HOME=${CRAIC_HOME}

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-navigation \
    ros-melodic-amcl \
    ros-melodic-map-server \
    ros-melodic-move-base \
    ros-melodic-teb-local-planner \
    ros-melodic-eband-local-planner \
    ros-melodic-robot-state-publisher \
    ros-melodic-joint-state-publisher \
    ros-melodic-tf2-geometry-msgs \
    ros-melodic-cv-bridge \
    ros-melodic-image-transport \
    python3-pip \
    python3-opencv \
    python3-catkin-tools \
    python-pip \
    python-opencv \
    tesseract-ocr \
    tesseract-ocr-chi-sim \
    libzbar0 \
    libopencv-dev \
    git \
    && rm -rf /var/lib/apt/lists/* \
    && pip2 install --no-cache-dir 'pytesseract==0.2.9' 'pyzbar==0.1.8'

WORKDIR ${CRAIC_HOME}
COPY . ${CRAIC_HOME}/

RUN find "${CRAIC_HOME}" -type f \( -name '*.yaml' -o -name '*.py' -o -name '*.cpp' -o -name '*.launch' \) \
      -exec grep -l '/home/zinn' {} + 2>/dev/null | while read -r f; do \
        sed -i 's|/home/zinn|/root|g' "$f"; \
      done || true

RUN mkdir -p /root/snapshots

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    cd ${CRAIC_HOME}/nav_sim_ws && catkin_make"

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    source ${CRAIC_HOME}/nav_sim_ws/devel/setup.bash && \
    cd ${CRAIC_HOME}/nav_real_ws && catkin_make"

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    cd ${CRAIC_HOME}/robot_ws && catkin_make"

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    cd ${CRAIC_HOME}/control_ws && catkin_make"

COPY docker/entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
