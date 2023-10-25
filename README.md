# RAMONES Glider (glider-ramones repo)
Repository for glider missions in the scope of the Ramones Project

## Installation
In the glider's internal Raspberry Pi, create a folder and clone the repo:

```shell
mkdir -p ~/catkin_ws_glider_ramones/src
git clone --recurse-submodules [repository_link] ~/catkin_ws_glider_ramones/src/.
```

To launch the code, simply run:

```shell
roslaunch glider_bringup start_vehicle.launch
```

Always remember to check which processes are being launched, in the file `~/catkin_ws_glider_ramones/src/glider_bringup/config/vehicles/glider/process.yaml`.