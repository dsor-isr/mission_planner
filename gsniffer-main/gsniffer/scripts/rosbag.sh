#!/bin/bash

source /home/pi/gsniffer_ws/devel/setup.bash

BAGS_FOLDER='/media/data/bags'
mkdir -p ${BAGS_FOLDER}
cd ${BAGS_FOLDER}

rosbag record -a --split --size 1024 --lz4
