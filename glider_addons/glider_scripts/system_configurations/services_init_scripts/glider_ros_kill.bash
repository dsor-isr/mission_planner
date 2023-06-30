#! /bin/sh

# Script to kill all ROS aplications
echo 'Killing rosbag record'

timeout 5 bash -c -- '
while pgrep record > /dev/null;
do
        pgrep record | xargs kill -SIGINT
        sleep 1;
done'

echo 'Killing remaining processes'
pgrep record | xargs kill -9

echo 'Killing ROS'
killall -SIGINT roslaunch &
killall -9 rosmaster &
killall -9 rosout