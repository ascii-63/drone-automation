#! /bin/bash
cd ~/drone-automation

#########

cd gcs-comm-service && docker compose up &
cd ..

#########

./build/cleaner

#########

source ~/.bashrc && roscore
rosrun control_pkg peripherals_node &
sleep 5
rosrun control_pkg log_node &
sleep 5

########

./build/mid-man