# 根据架构选择基础镜像
ARG TARGETARCH

FROM ubuntu:22.04 AS amd64
FROM arm64v8/ubuntu:22.04 AS arm64
FROM ${TARGETARCH} AS final

# 重新声明 ARG 
ARG TARGETARCH

# 创建 workspace
RUN mkdir -p /ros_ws
WORKDIR /ros_ws/

# 设置非交互式环境避免卡在时区选择
ENV DEBIAN_FRONTEND=noninteractive
# 设置默认时区
ENV TZ=Asia/Shanghai

# 设置线程数
ARG THREADS=16
ENV THREADS=${THREADS}

# 更换软件源为中科大源
RUN \
    if [ -f /etc/apt/sources.list ]; then \
        if [ "$TARGETARCH" = "arm64" ]; then \
            sed -i \
                -e 's@//ports.ubuntu.com/@//ports.ubuntu.com/ubuntu-ports/@g' \
                -e 's@//ports.ubuntu.com@//mirrors.ustc.edu.cn@g' \
                /etc/apt/sources.list; \
        else \
            sed -i 's@//.*archive.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list; \
        fi; \
    else \
        if [ "$TARGETARCH" = "arm64" ]; then \
            sed -i 's@//ports.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list.d/ubuntu.sources; \
        else \
            sed -i 's@//.*archive.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list.d/ubuntu.sources; \
        fi; \
    fi

# apt 依赖
RUN \
    apt update && apt install -y \
    # 一些常用工具
    nano htop rsync iproute2 \
    # ROS
    locales curl lsb-release \
    # 编译工具
    git cmake build-essential clangd g++ \
    # 依赖
    libeigen3-dev libceres-dev libopencv-dev libfmt-dev \
    # openvino
    wget gnupg software-properties-common \
    bash-completion  && \
    apt clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* && \
    # 设置语言环境
    locale-gen zh_CN.UTF-8 && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=zh_CN.UTF-8

ENV LANG=zh_CN.UTF-8
ENV LC_ALL=zh_CN.UTF-8

# 安装 ROS2 Humble 
RUN \
    # 安装密钥
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.ustc.edu.cn/ros2/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    # 安装 ROS2 Humble
    apt update && \
    apt install -y ros-humble-desktop ros-humble-foxglove-bridge python3-colcon-common-extensions python3-rosdep python3-argcomplete && \
    apt clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# openvino
# armd64架构下从源码编译安装，x86架构下使用Intel官方apt源安装
RUN \
    if [ "$TARGETARCH" = "arm64" ]; then \
        echo "Building OpenVINO from source for arm64..." && \
        git clone -b 2025.3.0 https://gitee.com/openvinotoolkit-prc/openvino.git /tmp/openvino && \
        cd /tmp/openvino && \
        chmod +x scripts/submodule_update_with_gitee.sh && \
        ./scripts/submodule_update_with_gitee.sh && \
        ./install_build_dependencies.sh && \
        mkdir build && cd build && \
        cmake -DCMAKE_BUILD_TYPE=Release .. && \
        make -j16 && \
        make install && \
        rm -rf /tmp/openvino ; \
    else \
        echo "Installing OpenVINO from Intel APT repository for amd64..." && \
        wget -O /tmp/intel-gpg-key.pub https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
        gpg --dearmor < /tmp/intel-gpg-key.pub > /etc/apt/trusted.gpg.d/intel-openvino.gpg && \
        echo "deb https://apt.repos.intel.com/openvino ubuntu22 main" > /etc/apt/sources.list.d/intel-openvino.list && \
        apt update && apt install -y openvino-2025.3.0 && \
        apt clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* ; \
    fi 

# rmvision
RUN \
    # rosdep
    mkdir -p /etc/ros/rosdep/sources.list.d/ && \
    curl -o /etc/ros/rosdep/sources.list.d/20-default.list https://mirrors.ustc.edu.cn/rosdistro/rosdep/sources.list.d/20-default.list && \
    sed -i 's#raw.githubusercontent.com/ros/rosdistro/master#mirrors.ustc.edu.cn/rosdistro#g' /etc/ros/rosdep/sources.list.d/20-default.list && \
    export ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml && \
    rosdep update && apt update && \
    # 安装rmvision依赖
    apt install -y ros-humble-asio-cmake-module \
    ros-humble-vision-opencv \
    ros-humble-camera-info-manager \
    ros-humble-camera-calibration-parsers \
    ros-humble-camera-calibration \
    ros-humble-serial-driver \
    ros-humble-udp-msgs \
    ros-humble-io-context \
    ros-humble-asio-cmake-module \
    ros-humble-ament-cmake-clang-format \
    ros-humble-angles \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-tf-transformations \
    ros-humble-xacro \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-plotjuggler \
    ros-humble-plotjuggler-ros \
    && apt clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# 添加环境变量 到 bashrc
RUN echo "if [ -f /ros_ws/install/setup.bash ]; then source /ros_ws/install/setup.bash; fi" >> /root/.bashrc \
    && echo "if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi" >> /root/.bashrc \
    && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /root/.bashrc \
    && echo "if [ -f /usr/share/bash-completion/bash_completion ]; then source /usr/share/bash-completion/bash_completion; fi" >> /root/.bashrc  \
    && echo 'export ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml' >> /root/.bashrc

# 安装Intel GPU驱动
RUN \
    if [ "$TARGETARCH" = "amd64" ]; then \
        echo "Installing Intel GPU Drivers for amd64..." && \
        # 添加Intel Graphics官方GPG密钥
        wget -qO - https://repositories.intel.com/gpu/intel-graphics.key | gpg --dearmor --output /usr/share/keyrings/intel-graphics.gpg && \
        # 添加Intel Graphics官方软件源
        echo "deb [arch=amd64 signed-by=/usr/share/keyrings/intel-graphics.gpg] https://repositories.intel.com/gpu/ubuntu jammy client" | tee /etc/apt/sources.list.d/intel-gpu-jammy.list && \
        apt update && \
        # 安装核心驱动组件：
        apt install -y \
            intel-opencl-icd \
            intel-level-zero-gpu \
            level-zero \
            ocl-icd-libopencl1 && \
        apt clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* ; \
    else \
        echo "Skipping Intel GPU Drivers for non-amd64 architecture" ; \
    fi

# 启动环境变量
RUN cat <<EOF > /rm_entrypoint.sh
#!/bin/bash
set -e

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
fi

if [ -f "/ros_ws/install/setup.bash" ]; then
    source "/ros_ws/install/setup.bash"
fi

if [ -f "/usr/local/setupvars.sh" ]; then
    source "/usr/local/setupvars.sh"
fi

exec "\$@"
EOF
RUN chmod +x /rm_entrypoint.sh
ENTRYPOINT ["/rm_entrypoint.sh"]

# 允许 Git 信任挂载的开发目录
RUN git config --global --add safe.directory /ros_ws