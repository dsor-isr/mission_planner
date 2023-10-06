#! /bin/bash

# Script to run all ROS aplications
source /home/ubuntu/.bashrc
source /home/ubuntu/catkin_ws_glider_ramones/devel/setup.bash
export ROS_BAG_FOLDER=/home/ubuntu
export ROS_IP=$HOSTNAME
# name=$HOSTNAME
# name=glider



#echo "Giving Leak Permissions"
#sudo chown 0.0 ${ROS_WORKSPACE}/devel/lib/parallel_port/parallel_port
#sudo chmod ug+s ${ROS_WORKSPACE}/devel/lib/parallel_port/parallel_port

echo "Sync clock with network time protocol "
#sudo ntpdate -bu ntpServer

echo "Run glider stack"
roslaunch glider_bringup start_vehicle.launch name:=glider