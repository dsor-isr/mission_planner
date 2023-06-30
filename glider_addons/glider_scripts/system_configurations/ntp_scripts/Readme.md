```

Tinkers: DSOR Group
         malaclemys  -> jquintas@gmail.com

Description: Scripts for configuring ntp server or client depending on the machine.
Date: 20200226

@~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~@
@         ?                                               @
@          __                                             @
@      ____||_                  ___                       @
@     (_______)            ,,  // \\                      @  
@      ____||_            (_,\/ \_/ \                     @
@     (_______)             \ \_/_\_/>                    @
@                            /_/  /_/         ______      @
@                                            |START |     @
@                                            |______|     @
@                                            |            @
@____________________________________________|____________@
```

## Purpose

The script will configure for you the network time protocol responsible for syncronizing all the clocks between all the machines in the glider network.

## ntp server

Please run the following line in terminal to configure the nto server:
```
bash config_ntp.sh server
```

This should be runned in the trials laptop only, because it is the one defined as server in the hosts file:
```
192.168.1.77	trials ntpServer
```

This will:
- install ntp
- replace the /etc/ntp.conf file with the ntp configuration @{CATKIN_ROOT}/catkin_ws/src/glider_vx/glider_scripts/system_configurations/**ntp_server.conf**.
- Restart NTP Server
- Configure Firewall so that client(s) can access NTP server

## ntp client

Every single vehicle should have the ntp client configured. The script is called while configuring the os when using *config_glider_os.bash* script.

This will:
- install ntpdate and ntp
- Disable the default Ubuntu systemd's timesyncd service
- replace the /etc/ntp.conf file with the ntp configuration @{CATKIN_ROOT}/catkin_ws/src/glider_vx/glider_scripts/system_configurations/**ntp_client.conf**.
- Restart NTP Server

## Extra details

For extra details read the Readme @{CATKIN_ROOT}/catkin_ws/src/glider_vx/glider_scripts/system_configurations/ntp_server_client/.