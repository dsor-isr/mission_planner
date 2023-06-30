#!/bin/bash


scripts_folder="/home/ubuntu/catkin_ws_real/src/glider_addons/glider_scripts/system_configurations/bullet_switch"

sudo gcc "$scripts_folder/parallel_port.c" -o parallel_port

sudo chown 0.0 parallel_port
sudo chmod ug+s parallel_port

sudo mv parallel_port /usr/bin/parallel_port

# This is no longer necessary. The service was replaced to the startup glider and to switch use the aliases
#sudo rm -f /etc/init.d/bullet
#sudo ln -s "$scripts_folder/bullet" /etc/init.d/bullet
#sudo update-rc.d bullet defaults
#sudo update-rc.d bullet enable
