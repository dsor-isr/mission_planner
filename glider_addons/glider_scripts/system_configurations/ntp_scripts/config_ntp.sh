#!/bin/bash

#Tinkers: DSOR Group
#         malaclemys  -> jquintas@gmail.com
#
#Descripton: bash script that installs ntp for client or server and copies 
#            the corresponding configuration file to /etc/ntp.conf
#Date: 20200226
#
#@~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~@
#@         ?                                               @
#@          __                                             @
#@      ____||_                  ___                       @
#@     (_______)            ,,  // \\                      @  
#@      ____||_            (_,\/ \_/ \                     @
#@     (_______)             \ \_/_\_/>                    @
#@                            /_/  /_/         ______      @
#@                                            |START |     @
#@                                            |______|     @
#@                                            |            @
#@____________________________________________|____________@
export GLIDER_SCRIPTS=$(find ${ROS_WORKSPACE}/src/ -type d -iname glider_scripts | head -n 1)

##################################################################################################
# @.@  Function for copying the desired ntp configuration to /etc folder                         #
##################################################################################################

copy_ntp_conf () {
    echo "###########################################"
    echo "#Copying ${ntp_file} to /etc/ntp.conf     #"
    echo "###########################################"
    
    echo ${ntp_file}
    sudo rm /etc/ntp.conf
    
    # *.* Go to script folder   
    cd "${0%/*}"
    # *.* Copy the configuration ntp_{server|client} -> /etc/ntp.conf
    sudo cp ${GLIDER_SCRIPTS}/system_configurations/ntp_server_client/${ntp_file} /etc/ntp.conf
}

###########################################################################################################
# @.@  Checks if the user wants to install and configure server or client, if none gives a helper message #
###########################################################################################################

if [ ${1} == "server" ] 
then
    ntp_file="ntp_${1}.conf"
    echo "##############################"
    echo "#Installing ntp for ntp ${1} #"
    echo "##############################"
    
    # *.* Install ntp for server
    sudo apt-get install ntp -y

    # *.* Calls function to place configuration in /etc folder
    copy_ntp_conf

    # *.* Restart NTP Server
    sudo service ntp restart
   
    # *.* Configure Firewall so that client(s) can access NTP server
    sudo ufw allow from any to any port 123 proto udp

elif [ ${1} == "client" ]
then
    ntp_file="ntp_${1}.conf"

    echo "##########################################"
    echo "#Installing ntpdate and ntp for ntp ${1} #"
    echo "##########################################"
    
    # *.* Install ntpdate and ntp for client
    sudo apt-get install ntpdate ntp -y

    # *.* Disable the default Ubuntu systemd's timesyncd service
    sudo timedatectl set-ntp off

    # *.* Calls function to place configuration in /etc folder
    copy_ntp_conf

    # *.* Restart ntp
    sudo service ntp restart
    
else
    # *.* The user has to say if it is client or server, any other option will retrieve the following helper message
    echo "Usage: bash config_ntp.sh {client|server}"
fi
