#!/bin/bash

echo "Starting ROS master..."
roscore &
sleep 3

echo "Starting rosbridge..."
roslaunch rosbridge_server rosbridge_websocket.launch &
sleep 2

echo "Starting Web Video Server on 8081..."
rosrun web_video_server web_video_server _port:=8081 &
sleep 2

echo "Starting Web UI on 3000..."
cd /web_app
npm start -- --port=3000
