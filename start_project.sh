#!/bin/bash

# 1. Start ROS core/launch in the background
echo "Starting ROS..."
# EDIT THIS LINE: Replace with your actual launch file
# Example: roslaunch my_drone_pkg main_mission.launch &
roslaunch rosbridge_server rosbridge_websocket.launch &

# 2. Start Web Video Server (if you use it for streaming)
rosrun web_video_server web_video_server &

# 3. Start the Web UI
echo "Starting Web UI..."
cd /web_app
npm start