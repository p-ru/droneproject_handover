#!/bin/bash

# 1. roscore
echo "Starting ROS master..."
roscore &
sleep 3

# 2. Launch Rosbridge (Internal include)
echo "Starting Rosbridge..."
roslaunch rosbridge_server rosbridge_websocket.launch &

# 3. Launch Web Video Server (Setting the param via command line)
echo "Starting Web Video Server on port 9092..."
rosrun web_video_server web_video_server _port:=9092 &

# 4. Launch RViz
echo "Starting RViz..."
rosrun rviz rviz -d $(rospack find tagslam)/example/tagslam_example.rviz &

echo "Starting Web UI..."
cd /web_app
npm start &

Groot &
echo "All systems launched. Container is active."

# This waits for all background processes started in this script
wait
# echo "Launching mission..."
# roslaunch behaviortree_ros run_waypoints.launch

# echo "Launching mission..."
# roslaunch behaviortree_ros run_waypoints.launch
