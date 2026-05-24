# base image
FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG USE_MIRROR=0  # 临时禁用镜像源
ARG MIRROR_URL="archive.ubuntu.com"
ARG INSTALL_GPP13=1
ARG USERNAME=agi
ARG USER_UID=1001
ARG USER_GID=1001
ARG ROS_MIRROR="tuna"

ENV DEBIAN_FRONTEND=${DEBIAN_FRONTEND}
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV TZ=Etc/UTC

# 使用原始 Ubuntu 源，增加重试机制
RUN for i in 1 2 3; do \
      apt-get update && break || sleep 15; \
    done && \
    for i in 1 2 3; do \
      apt-get install -y --no-install-recommends \
      curl gnupg lsb-release locales wget sudo ca-certificates python3-pip && break || sleep 15; \
    done && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# 安装 ROS2 Humble（使用 TUNA 镜像）
RUN for i in 1 2 3; do \
      curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && break || sleep 15; \
    done && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list && \
    for i in 1 2 3; do \
      apt-get update && \
      apt-get install -y --no-install-recommends \
        ros-humble-desktop python3-argcomplete python3-colcon-common-extensions \
        ros-humble-grid-map-msgs ros-humble-grid-map-ros ros-humble-rmw-cyclonedds-cpp \
        libgflags-dev libscrypt-dev liboctomap-dev liborocos-kdl-dev \
        libacl1-dev python3-pip cmake unzip && break || sleep 15; \
    done && \
    rm -rf /var/lib/apt/lists/*

# 创建用户
RUN groupadd -g ${USER_GID} ${USERNAME} || true && \
    useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 创建目录并设置权限
RUN mkdir -p /agibot/data/var/ && \
    chown -R ${USERNAME}:${USERNAME} /agibot

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# 设置环境变量
RUN echo "export PATH=/home/${USERNAME}/.local/bin:$PATH" >> /home/${USERNAME}/.bashrc

RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc

RUN echo "export ROS_DOMAIN_ID=20" >> /home/${USERNAME}/.bashrc

RUN echo "source /home/${USERNAME}/x2_deploy_workspace/aimdk_msgs/share/aimdk_msgs/local_setup.bash" >> /home/${USERNAME}/.bashrc

CMD ["/bin/bash"]
