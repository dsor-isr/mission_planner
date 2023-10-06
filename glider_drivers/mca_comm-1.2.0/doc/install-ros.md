# Install ROS Noetic

Follow the instructions in http://wiki.ros.org/noetic/Installation/Ubuntu

## Setup ROS Sources

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

## Install ROS Packages

```bash
sudo apt update
sudo apt install ros-noetic-ros-base
```

## Environment Setup

Add sourcing to the shell startup script as described [here](http://wiki.ros.org/noetic/Installation/Ubuntu#noetic.2FInstallation.2FDebEnvironment.Environment_setup):

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

## Install dependencies for building packages

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-rosdep
sudo rosdep init
rosdep update
```
