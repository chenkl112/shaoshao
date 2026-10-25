FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake build-essential python3-dev python3-colcon-common-extensions \
    python3-numpy python3-scipy python3-pil python3-yaml python3-serial python3-venv libeigen3-dev libpcl-dev \
    libcurl4-openssl-dev ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-rviz2 ros-humble-pcl-ros ros-humble-pcl-conversions \
    ros-humble-octomap-server ros-humble-octomap-ros ros-humble-tf2-ros \
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv --system-site-packages /opt/localization-venv \
    && /opt/localization-venv/bin/pip install --no-cache-dir small-gicp==1.0.1
ENV PATH="/opt/localization-venv/bin:${PATH}"

ARG LIVOX_SDK_COMMIT=c0796f04c143143899c87a773d9f6b7136453c0b
RUN git init /opt/Livox-SDK2 \
    && git -C /opt/Livox-SDK2 fetch --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git ${LIVOX_SDK_COMMIT} \
    && git -C /opt/Livox-SDK2 checkout --detach FETCH_HEAD \
    && cmake -S /opt/Livox-SDK2 -B /opt/Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /opt/Livox-SDK2/build -j4 \
    && cmake --install /opt/Livox-SDK2/build \
    && ldconfig

WORKDIR /workspace
