FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

# Create non-root user matching host UID
ARG USERNAME=matt
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install system dependencies
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
    python3-osrf-pycommon 

RUN sudo apt-add-repository ppa:borglab/gtsam-release-4.1 \
    && apt-get update && apt-get install -y \
    libgtsam-dev libgtsam-unstable-dev \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*


# --- 3. INSTALL NODE.JS (For the Web UI) ---
# Installing Node 18 LTS
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash - \
    && apt-get install -y nodejs

# --- 4. BUILD ROS WORKSPACE ---
WORKDIR /ros_ws

# Copy ONLY the src folder first
COPY src/ ./src/

# Source ROS and Build
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

# --- 5. SETUP WEB UI ---
WORKDIR /web_app

# Copy package.json first to leverage Docker caching
COPY web_ui/package*.json ./

# Install Web Dependencies
RUN npm install

# Copy the rest of the web source code
COPY web_ui/ .

# --- 6. FINAL SETUP ---
# Copy scripts
COPY entrypoint.sh /entrypoint.sh
COPY start_project.sh /start_project.sh
RUN chmod +x /entrypoint.sh /start_project.sh

# Environment Setup
WORKDIR /ros_ws
ENTRYPOINT ["/entrypoint.sh"]

# Default Command: Runs the start script
CMD ["/start_project.sh"]