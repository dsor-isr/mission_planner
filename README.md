<<<<<<< HEAD
# RAMONES Glider (glider-ramones repo)
Repository for glider missions in the scope of the Ramones Project

## Installation
In the glider's internal Raspberry Pi, create a folder and clone the repo:

```shell
mkdir -p ~/catkin_ws_glider_ramones/src
git clone --recurse-submodules [repository_link] ~/catkin_ws_glider_ramones/src/.
```

Follow the README at the gsniffer submodule. Remember to give the correct permissions to the installed libraries during the guide before compiling:

```shell
sudo chmod a+r /usr/local/include/WinTypes.h
sudo chmod a+r /usr/local/include/ftd2xx.h
```

To launch the code, simply run:

```shell
roslaunch glider_bringup start_vehicle.launch
```

Always remember to check which processes are being launched, in the file `~/catkin_ws_glider_ramones/src/glider_bringup/config/vehicles/glider/process.yaml`.
=======
# mission_planner
Repository for creation and maintenance of Path Following missions via acoustic communications.
>>>>>>> 13256eb0a834756804b5753c1eb961cb62224267
