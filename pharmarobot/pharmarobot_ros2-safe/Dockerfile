FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Base Ubuntu tools only.
# Universe must be enabled before installing ROS development packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    git \
    nano \
    python3-pip \
    libboost-thread-dev \
    psmisc \
    && add-apt-repository -y universe \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Add the ROS 2 Humble repository.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2.list

# Install ROS and the persistent joystick dependency.
# rosdep is not required because this image installs dependencies explicitly.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-humble-desktop \
    ros-humble-joy-linux \
    ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws

COPY ./src /ros_ws/src/

RUN . /opt/ros/humble/setup.sh \
    && colcon build \
       --symlink-install \
       --packages-up-to \
         serial \
         sllidar_ros2 \
         joy_to_cmdvel \
         command_arbiter \
         roboteq_ros2_driver \
         teleop_pharma

RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    'if [ -f /ros_ws/install/setup.bash ]; then source /ros_ws/install/setup.bash; fi' \
    >> /root/.bashrc

CMD ["bash", "-lc", "tail -f /dev/null"]
