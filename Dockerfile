FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

# --- 1. INSTALL SYSTEM DEPENDENCIES ---
# No 'sudo' needed, we are root.
RUN apt-get update && apt-get install -y \
    libfmt-dev \
    git \
    ros-noetic-behaviortree-cpp-v3 \
    qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    ros-noetic-rqt-robot-plugins \
    ros-noetic-image-transport-plugins \
    ros-noetic-rosbridge-suite \
    ros-noetic-web-video-server \
    ros-noetic-image-geometry \
    software-properties-common \
    python3-vcstool \
    python3-osrf-pycommon \
    curl

# Add GTSAM PPA (No sudo needed)
RUN add-apt-repository ppa:borglab/gtsam-release-4.1 \
    && apt-get update && apt-get install -y \
    libgtsam-dev libgtsam-unstable-dev \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# --- 2. INSTALL NODE.JS ---
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash - \
    && apt-get install -y nodejs

# --- 3. BUILD Groot ---
WORKDIR /Groot
# Copy directly (Root owns it by default)
COPY Groot/ .

# Build and Install
# Removed 'sudo' from cmake install
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --parallel $(nproc) \
    && cmake --install build

# --- 4. BUILD ROS WORKSPACE ---
WORKDIR /ros_ws

# Copy src
COPY src/ ./src/

# Source and Build
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

# --- 5. SETUP WEB UI ---
WORKDIR /web_app

# Copy package.json
COPY web_ui/package*.json ./

# Install Dependencies
# NPM warns about running as root, but it works fine in Docker
RUN npm install

# Copy source
COPY web_ui/ .

# --- 6. FINAL SETUP ---
COPY entrypoint.sh /entrypoint.sh
COPY start_project.sh /start_project.sh
RUN chmod +x /entrypoint.sh /start_project.sh

# Source workspace in ROOT's bashrc
RUN echo "source /ros_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /ros_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/start_project.sh"]