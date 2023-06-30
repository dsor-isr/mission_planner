```

Tinkers: DSOR Group
         malaclemys  -> jquintas@gmail.com

Descripton: Udev dope rules
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

## About udev rules in a vehicle

Udev rules determine how to identify devices and how to assign a name that is persistent through reboots or disk changes. When Udev receives a device event, it matches the configured rules against the device attributes in sysfs to identify the device. [source](https://www.thegeekdiary.com/beginners-guide-to-udev-in-linux/)

This is handy in ros code to define the port of a sensor. This way we can share the code in multiple vehicles and we just need to worry about consistence in udev rules.

## Example of udev rules (muned)

```
# USB Ports -> you can connect the following devices to any usb port
# VectorNav imu
# sensore pressure
# bluetooth

SUBSYSTEM=="tty", SUBSYSTEMS=="usb", DRIVERS=="usb", ATTRS{serial}=="FTXW9TH7", MODE="0666", SYMLINK+="medusa/imu"
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", DRIVERS=="usb", ATTRS{serial}=="FT08WOO0", MODE="0666", TAG+="uaccess",  SYMLINK+="medusa/inside_pressure"
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", DRIVERS=="usb", ATTRS{serial}=="A904KOQA", MODE="0666", SYMLINK+="medusa/bluetooth"

SUBSYSTEM=="tty", SUBSYSTEMS=="usb", DRIVERS=="usb", ATTRS{serial}=="FT08LHRT", MODE="0666", SYMLINK+="medusa/depth_cell"

# Thrusters
SUBSYSTEMS=="usb", ATTRS{idVendor}=="f4d8", ATTRS{idProduct}=="0001", SYMLINK+="medusa/thrusters_usb", GROUP="dialout", MODE="777"


# Serial ports -> you should keep the current setup
# plug each device to the current ports
KERNEL=="ttyS4", SYMLINK="medusa/gps"
KERNEL=="ttyS2", SYMLINK="medusa/altimeter"
KERNEL=="ttyS3", SYMLINK="medusa/batmonit"
KERNEL=="ttyS1", SYMLINK="medusa/dvl"

```

## How to add Udev Rules

This is done by running the following commands at the terminal:

```
#Linking udev rules
name=$HOSTNAME
sudo rm -f /etc/udev/rules.d/50-udev.rules
sudo ln -s "${CATKIN_ROOT}/catkin_ws/src/medusa_vx/medusa_scripts/system_configurations/udev_rules/${name}_50-udev.rules" /etc/udev/rules.d/50-udev.rules
```

 The previous commands are present *@${CATKIN_ROOT}/catkin_ws/src/medusa_vx/medusa_scripts/tinker_tools/installation_scripts/config_medusa_os.sh*. You should run this script when configuring a new vehicle.



## How to reload udev rules

```
udevadm control --reload-rules && udevadm trigger
```
[source](https://unix.stackexchange.com/questions/39370/how-to-reload-udev-rules-without-reboot)
[another source](https://unix.stackexchange.com/questions/39370/how-to-reload-udev-rules-without-reboot/39485#39485)

 ## **Get Info about the device**

udevadm info -a -p $(udevadm info -q path -n <devpath>) -> [more here](https://www.clearpathrobotics.com/assets/guides/ros/Udev%20Rules.html)
udevadm info --name=/dev/ttyS1 --attribute-walk -> [more here](https://wiki.debian.org/udev)

## **udev rules location file**
[udev folder](https://askubuntu.com/questions/978552/how-do-i-make-libusb-work-as-non-root)

## **Udev rules usb examples**

See this [USB serial example](https://indilib.org/support/tutorials/157-persistent-serial-port-mapping.html) and complement it with the info from the next link, otherwise it won't work 
**[This dude saved me](https://askubuntu.com/questions/1021547/writing-udev-rule-for-usb-device)**
[Another example](https://ea4tx.com/en/faqs/mapeo-de-puertos-por-numero-de-serie-linux/)

## Tutorial (general stuff)
[Tutorial](http://www.reactivated.net/writing_udev_rules.html#syntax)

## General udev rules examples

[Example 1](https://linux-tips.com/t/how-to-set-generic-udev-rules-for-devices-with-different-serials/555)
[Example 2](https://gist.github.com/gdamjan/1434504)
[Example 3](https://wiki.archlinux.org/index.php/Talk:Udev)
