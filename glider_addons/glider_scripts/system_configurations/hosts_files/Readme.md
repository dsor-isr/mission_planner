```

Tinkers: DSOR Group
         malaclemys  -> jquintas@gmail.com

Descripton: Documentation about hosts_files
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

## Hosts_append_file

File with all the hosts related to the glider_stack:
- vehicles
- modems
- users

## hosts_cpy.sh


Script for adding the hosts to the system /etc/hosts. In vehicles this is always executed with the startup service: 
 - **gliderStartUp** -> @services_init_scripts.

For running it manually execute the following command in terminal:

```
sudo bash ${CATKIN_ROOT}/catkin_ws/src/glider_vx/glider_scripts/system_configurations/hosts_files/hosts_cpy.sh
```
