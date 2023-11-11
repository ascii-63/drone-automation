#! /bin/bash
#############################################################

/home/pino/drone-automation/build/cleaner

#############################################################

# echo "gcs-comm-service launching"
# cd /home/pino/drone-automation/gcs-comm-service && sudo docker-compose up &
# sleep 5

#############################################################

echo "source .bashrc and roscore..."

source /home/pino/.bashrc
sleep 1

#############################################################

echo "launch mavros and geometric_controller..."

# roslaunch mavros px4.launch fcu_url:="udp://:14540@localhost:14557" &
# roslaunch px4 mavros_posix_sitl.launch &
roslaunch mavros px4.launch &
sleep 10

roslaunch geometric_controller automatic.launch &
sleep 10

#############################################################

echo "run peripherals_node and log_node..."

rosrun control_pkg peripherals_node &
sleep 5

rosrun control_pkg log_node &
sleep 5

#############################################################

echo "launch realsense2 and spinnaker driver for camera..."

# roslaunch realsense2_camera rs_camera.launch &
# sleep 10

roslaunch spinnaker_camera_driver color_cam.launch &
sleep 10

#############################################################

/home/pino/drone-automation/build/mid-man