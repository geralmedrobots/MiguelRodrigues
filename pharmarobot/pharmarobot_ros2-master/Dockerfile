# Base image with Ubuntu 22.04
FROM ubuntu:22.04

ARG BUILD_MODE=development
ENV BUILD_MODE="${BUILD_MODE}"
ENV DEBIAN_FRONTEND=noninteractive

# Set locale
RUN apt-get update && apt-get install -y locales \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    software-properties-common \
    nano

# Mode-specific installations
RUN if [ "${BUILD_MODE}" = "production" ]; then \
      echo "Installing production packages"; \
    else \
      echo "Installing dev tools including git, vim"; \
      apt-get install -y git vim htop; \
    fi

# Add the ROS 2 GPG key and source list (modern way)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y ros-humble-desktop

# Source ROS 2 environment on every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Install colcon and other ROS tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
 && pip3 install -U argcomplete

# Install additional ROS 2 packages
RUN apt-get update && apt-get install -y \
    libspnav-dev \
    libbluetooth-dev \
    libcwiid-dev \
    ros-humble-geographic-msgs \
    ros-humble-test-msgs \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-realsense2-camera-msgs \
    ros-humble-sick-safetyscanners2-interfaces \
    ros-humble-sick-safetyscanners-base \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Setup workspace
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws

# Only for production: Copy workspace and build
RUN if [ "${BUILD_MODE}" = "production" ]; then \
      echo "Setting up production environment"; \
      # This is a placeholder - actual COPY will happen after this in production builds \
      echo "Workspace will be copied at build time"; \
    else \
      echo "Setting up development environment"; \
      echo "Workspace will be mounted at runtime"; \
    fi


# COPY instructions that only apply to production builds
# These will be skipped in development mode due to build context
COPY ./src /ros_ws/src/

# Build workspace only in production mode
RUN if [ "${BUILD_MODE}" = "production" ]; then \
      . /opt/ros/humble/setup.sh && \
      colcon build --symlink-install; \
    fi

# Setup ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /ros_ws/install/setup.bash ]; then source /ros_ws/install/setup.bash; fi" >> /root/.bashrc

# Default command
CMD ["/bin/bash"]
