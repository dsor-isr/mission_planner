```

Tinkers: DSOR Group
         malaclemys  -> jquintas@gmail.com

Descripton: Documentation of glider stack startup services 
            Services that automatically start with vehicle boot. 
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

## Add Services

This is done with the bash script **config_glider_os.sh** *@glider_scripts/tinker_tools/installation_scripts*. Only do this in vehicles, you don't want to start ROS with your personal pc.

``` bash
#Create gliderStartup
sudo rm -f /etc/init.d/gliderStartUp
sudo ln -s "${glider_SCRIPTS}/system_configurations/services_init_scripts/gliderStartUp" /etc/init.d/gliderStartUp
sudo update-rc.d gliderStartUp defaults 99
```

## gliderStartUP Service

### While booting
- Syncronizes hosts file
- Gives leaks permission (leak detector)
- Starts up ROS (*@glider-scripts/tinker_tools/ros_scripts/startup_glider_stack/glider_ros_start*)
    - alias **glider_start** @glider_scripts/easy_alias/glider_personal_alias/mred||mblack||myellow_alias.sh

### More operations
- Stop ROS (*@glider_scripts/tinker_tools/ros_scripts/kill_glider_stack/glider_ros_kill*)
    - alias **glider_stop** @glider_scripts/easy_alias/glider_personal_alias/mred||mblack||myellow_alias.sh

- Restart ROS (*glider-scripts/tinker_tools/ros_scripts/kill_glider_stack/glider_ros_kill*)
    - alias **glider_restart** @glider_scripts/easy_alias/glider_personal_alias/mred||mblack||myellow_alias.sh
