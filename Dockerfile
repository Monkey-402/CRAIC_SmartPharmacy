# CRAIC — Ubuntu 18.04 + ROS Melodic
# 默认构建仿真导航；可选 BUILD_CONTROL_WS=1 编译 control_ws（含 YOLO / PaddleOCR）
#
# 构建:
#   docker build -t craic:melodic /path/to/craic
#   docker build -t craic:melodic --build-arg BUILD_CONTROL_WS=1 /path/to/craic
#
# 仿真导航:
#   docker compose run --rm craic roslaunch car_sim nav_sim.launch
#   # 或
#   docker run --rm -it --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix craic:melodic \
#     roslaunch car_sim nav_sim.launch

FROM osrf/ros:melodic-desktop-full

ARG BUILD_CONTROL_WS=0
ARG CRAIC_HOME=/root/craic

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONIOENCODING=utf-8 \
    ROS_DISTRO=melodic \
    CRAIC_HOME=${CRAIC_HOME}

# 导航 / 仿真依赖（desktop-full 已含 Gazebo、RViz 等）
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
    libopencv-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# control_ws：OpenCV 4 在 18.04 官方源中不可用，构建时放宽为 OpenCV 3（API 足够）
# Python 视觉依赖仅在 BUILD_CONTROL_WS=1 时安装
RUN if [ "$BUILD_CONTROL_WS" = "1" ]; then \
      pip3 install --no-cache-dir --upgrade "pip<24" setuptools wheel && \
      pip3 install --no-cache-dir \
        "numpy<1.24" \
        onnxruntime \
        "paddlepaddle==2.4.2" \
        "paddleocr==2.6.1.3"; \
    fi

WORKDIR ${CRAIC_HOME}
COPY . ${CRAIC_HOME}/

# 修正仓库内写死的本机路径 -> 容器内路径
RUN find "${CRAIC_HOME}" -type f \( -name '*.yaml' -o -name '*.py' -o -name '*.cpp' -o -name '*.launch' \) \
      -exec grep -l '/home/zinn' {} + 2>/dev/null | while read -r f; do \
        sed -i 's|/home/zinn|/root|g' "$f"; \
      done || true

RUN mkdir -p /root/snapshots

# ---------- catkin 编译 ----------
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    cd ${CRAIC_HOME}/nav_sim_ws && catkin_make"

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    source ${CRAIC_HOME}/nav_sim_ws/devel/setup.bash && \
    cd ${CRAIC_HOME}/nav_real_ws && catkin_make"

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    cd ${CRAIC_HOME}/robot_ws && catkin_make"

RUN if [ "$BUILD_CONTROL_WS" = "1" ]; then \
      sed -i 's/find_package(OpenCV 4 REQUIRED)/find_package(OpenCV 3 REQUIRED)/' \
        ${CRAIC_HOME}/control_ws/src/move_nav/CMakeLists.txt && \
      /bin/bash -c "source /opt/ros/melodic/setup.bash && \
        cd ${CRAIC_HOME}/control_ws && catkin_make"; \
    fi

COPY docker/entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
