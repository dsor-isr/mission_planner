```

Tinkers: DSOR Group
         malaclemys  -> jquintas@gmail.com

Descripton: Documentation about ssh banners
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

## SSH Banners

Vehicle ssh banners for vehicles or pc labs.
In vehicles this is added by using the script *glider_scripts/tinker_tools/installation_scripts/config_glider_os.sh*.


If you want to manually add a banner in a machine please run the following command in terminal after running the script *glider_scripts/tinker_tools/installation_scripts/config_glider_ros.sh*:

```
# *.* via symbolic link 

sudo rm -f /etc/motd
sudo ln -s "${CATKIN_ROOT}/catkin_ws/src/glider_vx/glider_scripts/system_configurations/motd_banners/motd" /etc/motd

# *.* or copy the file to /etc/motd

sudo cp ${CATKIN_ROOT}/catkin_ws/src/glider_vx/glider_scripts/system_configurations/motd_banners/motd /etc/motd
```

## Possible banners

### motd (original)
```
¸.·´¯`·.´¯`·.¸¸.·´¯`·.¸><(((º> glider powered by ISR/IST
```

### motd_glider_genX (default)
```
################# Welcome to #######################
    __  ___         __                      __                          
   /  |/  /__  ____/ /_  ___________ _     _||____       
  / /|_/ / _ \/ __  / / / / ___/ __ `/    (_______)      
 / /  / /  __/ /_/ / /_/ (__  ) /_/ /      _||____       
/_/  /_/\___/\__,_/\__,_/____/\__,_/      (_______)      

############### Enjoy the ride #####################
```

### motd_glider_genX (for glider deep sea-mds)

```
```
 _    _                            ____  ____
|  \/  | ___  __| |_   _ ___  __ _|  _ \/ ___|
| |\/| |/ _ \/ _` | | | / __|/ _` | | | \___ \
| |  | |  __/ (_| | |_| \__ \ (_| | |_| /___) |
|_|  |_|\___|\__,_|\__,_|___/\__,_|____/|____/
```

