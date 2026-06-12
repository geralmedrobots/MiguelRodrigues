# ROS 2 Jazzy + RViz on Ubuntu 24.04 for Docker Desktop on Windows
# Build context: this folder

FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_INDIRECT=0
ENV DISPLAY=host.docker.internal:0.0
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Developer and runtime tools commonly needed for ROS 2 workspaces and RViz diagnostics
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    git \
    nano \
    vim \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-jazzy-rmw-cyclonedds-cpp \
    mesa-utils \
    x11-apps \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1001
ARG USER_GID=1001

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} -s /bin/bash \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /home/${USERNAME}/ros2_ws/install/setup.bash ]; then source /home/${USERNAME}/ros2_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc \
    && chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.bashrc

WORKDIR /home/${USERNAME}/ros2_ws
RUN mkdir -p src

# Source ROS 2 automatically in every interactive shell
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /home/${USERNAME}/ros2_ws/install/setup.bash ]; then source /home/${USERNAME}/ros2_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc \
    && echo "export DISPLAY=${DISPLAY}" >> /home/${USERNAME}/.bashrc \
    && echo "export QT_X11_NO_MITSHM=1" >> /home/${USERNAME}/.bashrc \
    && echo "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >> /home/${USERNAME}/.bashrc

CMD ["bash"]
