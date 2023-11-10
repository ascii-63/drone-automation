#! /bin/bash

echo "source .bashrc and roscore..."

source ~/.bashrc

#############################################################

echo "launch mavros and geometric_controller..."

roslaunch mavros px4.launch &
sleep 5

roslaunch geometric_controller automatic.launch &
sleep 5

#############################################################

echo "run peripherals_node and log_node..."

rosrun control_pkg peripherals_node &
sleep 5

rosrun control_pkg log_node &
sleep 5

#############################################################

echo "launch realsense2 and spinnaker driver for camera..."

roslaunch realsense2_camera rs_camera.launch &
sleep 5

roslaunch spinnaker_camera_driver color_cam.launch &
sleep 5