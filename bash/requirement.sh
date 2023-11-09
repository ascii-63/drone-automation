#! /bin/bash

source ~/.bashrc

roslaunch mavros px4.launch &
sleep 5

roslaunch geometric_controller automatic.launch &
sleep 5

#############################################################

# Realsense2 (D455 and T265)
roslaunch realsense2_camera rs_camera.launch &
sleep 5
# echo "SKIP REALSENSE PACKAGE: No REALSENSE camera device connected."

# Spinnaker (FLIR)
roslaunch spinnaker_camera_driver color_cam.launch &
sleep 5