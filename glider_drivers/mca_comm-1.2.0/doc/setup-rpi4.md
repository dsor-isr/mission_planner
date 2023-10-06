# Setup RPi 4

## Install Ubuntu Server

Install Ubuntu Server 64-bit on RPi 4 following the instructions
[here](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#2-prepare-the-sd-card), which are
described in brief below.

Install `rpi-imager`:
```bash
wget https://downloads.raspberrypi.org/imager/imager_latest_amd64.deb
sudo apt install ./imager_latest_amd64.deb
```

Run it with `sudo rpi-imager` and select OS and storage. Choose the latest Ubuntu Server 20.\* LTS for ROS noetic or
ROS2 galactic, and 22.\* for newer ROS2 distributions.

![Other General Purpose OS](https://user-images.githubusercontent.com/382167/151121004-32968b18-8a0a-4550-b6f8-6d424f23d65b.png)
![Ubuntu](https://user-images.githubusercontent.com/382167/151121007-d28ae3a5-410f-4774-be53-376f9ae6d7b8.png)
![Ubuntu Server 20.04.03 LTS 64-bit](https://user-images.githubusercontent.com/382167/151121010-a2bc1626-23b8-47d7-8f01-c6e680074e13.png)

![Rasperry Pi Imager GUI](https://user-images.githubusercontent.com/382167/151121917-5c635d61-8791-4369-90cc-9545d99af074.png)

Configure the image to enable SSH and set the user `pi` and password `raspberry`.

Then press **WRITE**. It'd download and burn the image into the SD card:

![Ubuntu Server image successfully written to SD card](https://user-images.githubusercontent.com/382167/140382856-2e50c858-ce9b-45d3-8493-a447e57f3241.png)

After that, setup the network connection following the instructions
[here](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#3-wifi-or-ethernet).

* For the WiFi setup, add it to `sudo vim /etc/netplan/50-cloud-init.yaml` and then run:

``` bash
sudo netplan apply
sudo reboot
```

* For Ethernet nothing needs to be done.

:point_right: It's recommended to connect to a 2.4GHz WiFi network, because a 5GHz WiFi network might drop due to the
RPi 4 heat sinks.

Figure out the IP address using the router, and ssh in with:
```bash
ssh -A pi@192.168.0.25
```
