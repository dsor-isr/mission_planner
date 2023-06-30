```
Tinkers: DSOR Group
         malaclemys  -> jquintas@gmail.com

Descripton: Configuration of ntp server and client
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

# Introduction [source](https://vitux.com/how-to-install-ntp-server-and-client-on-ubuntu/)

NTP or Network Time Protocol is a protocol that is used to synchronize all system clocks in a network to use the same time. When we use the term NTP, we are referring to the protocol itself and also the client and server programs running on the networked computers. NTP belongs to the traditional TCP/IP protocol suite and can easily be classified as one of its oldest parts.

When you are initially setting up the clock, it takes six exchanges within 5 to 10 minutes before the clock is set up. Once the clocks in a network are synchronized, the client(s) update their clocks with the server once every 10 minutes. This is usually done through a single exchange of message(transaction). These transactions use port number 123 of your system.

In this article, we will describe a step-by-step procedure on how to:

Install and configure the NTP server on a Ubuntu machine.
Configure the NTP Client to be time synced with the server.

# NTP install and configuration in glider stack

## Install NTP server (only need to do this in trials pcs)

Run the following bash script:
```
sudo bash ${CATKIN_ROOT}/catkin_ws/src/glider_vx/glider_scripts/tinker_tools/ntp_scripts/config_ntp.sh **server**
```

This will:
- install ntp
- replace the /etc/ntp.conf file with the ntp configuration @{CATKIN_ROOT}/catkin_ws/src/glider_vx/glider_scripts/system_configurations/ntp_server_client/**ntp_server.conf**.
- Restart NTP Server
- Configure Firewall so that client(s) can access NTP server

## Install NTP client (all vehicles)

This is done using the **config_glider_os.sh** @{CATKIN_ROOT}/catkin_ws/src/glider_vx/glider_scripts/tinker_tools/installation_scripts. The script besides other things will run the following bash script:

```
sudo bash ${CATKIN_ROOT}/catkin_ws/src/glider_vx/glider_scripts/tinker_tools/ntp_scripts/config_ntp.sh **client**
```

This will:
- install ntpdate and ntp
- Disable the default Ubuntu systemd's timesyncd service
- replace the /etc/ntp.conf file with the ntp configuration @{CATKIN_ROOT}/catkin_ws/src/glider_vx/glider_scripts/system_configurations/ntp_server_client/**ntp_client.conf**.
- Restart NTP Server

## Hosts - Specify IP and hostname of the NTP server in the hosts file
You should have the following @glider_scripts/system_configurations/hosts_files/hosts_append_file and consequently in /etc/hosts:
```
192.168.1.77	trials ntpServer
```

# Details about NTP configuration files and how to install manually

## Install and configure NTP Server on the host computer

Follow these steps in order to install the NTP server on your host Ubuntu machine:

Note: We are using the Ubuntu command line, the Terminal, in order to install and configure NTP. You can open the Terminal application either through the application launcher search or the Ctrl+Alt+T shortcut.

### Step 1: Install NTP Server with apt-get

Please run the following command as sudo in order to install NTP server daemon from the APT repositories:
```
$ sudo apt-get install ntp -y
```
### Step 2: Verify installation (optional)

You can verify your NTP installation and also check the version number by running the following command in your Terminal:

```
$ sntp --version
```

### Step 3: Switch to an NTP server pool closest to your location

When you install the NTP server, it is mostly configured to fetch proper time. However, you can switch the server pool to the ones closest to your location. This includes making some changes in the /etc/ntp.conf file.

Open the file in the nano editor as sudo by running this following command:

```
sudo vim /etc/ntp.conf
```

In this file, you will be able to see a pool list. We have highlighted this list in the above image. The task here is to replace this pool list by a pool of time servers closest to your location. The pol.ntp.org project provides reliable NTP service from a big cluster of time servers. To choose a pool list according to your location, visit the following page:

https://support.ntp.org/bin/view/Servers/NTPPoolServers

In our case we want a pool list from [Portugal](https://www.pool.ntp.org/zone/pt). So edit/replace the following:
```
server 0.ubuntu.pool.ntp.org
server 1.ubuntu.pool.ntp.org
server 2.ubuntu.pool.ntp.org
server 3.ubuntu.pool.ntp.org
```

with:

```
server 3.pt.pool.ntp.org
server 0.europe.pool.ntp.org
server 3.europe.pool.ntp.org
```

### Step 4: Restart the NTP server
In order for the above changes to take effect, you need to restart the NTP server. Run the following command as sudo in order to do so:
```
sudo service ntp restart
```

### Step 6: Verify that the NTP Server is running 
Now, check the status of the NTP service through the following command:
```
sudo service ntp status
```
### Step 7: Configure Firewall so that client(s) can access NTP server
Finally, it is time to configure your system’s UFW firewall so that incoming connections can access the NTP server at UDP Port number 123.

Run the following command as sudo to open port 123 for incoming traffic:

```
sudo ufw allow from any to any port 123 proto udp
```

## Configure NTP Client to be Time Synced with the NTP Server

### Step 1: Install ntpdate
The ntpdate command will let you manually check your connection configuration with the NTP-server. Open the Terminal application on the client machine and enter the following command as sudo:
```
sudo apt-get install ntpdate
```

### Step 2: Specify IP and hostname of the NTP server in the hosts file

For your NTP server to be resolved by a hostname in your client machine, you need to configure your /etc/hosts file.

Open the hosts file as sudo in the nano editor by entering the following command:

```
sudo vim /etc/hosts
```

Now add your NTP server’s IP and specify a hostname as follows in this file:

```
192.168.x.xx NTP-server-host
```

### Step 3: Check if the client machine’s time is synchronized with NTP server

The following ntpdate command will let you manually check if time is synchronized between the client and server systems:

$ sudo ntpdate NTP-server-host
The output should ideally show a time offset between the two systems.

### Step 4: Disable the systemd timesyncd service on the client
Because we want our client to sync time with the NTP server, let us disable the timesyncd service on the client machine.
Enter the following command to do so:
```
sudo timedatectl set-ntp off
```

### Step 5: Install NTP on your client
Run the following command as sudo in order to install NTP on your client machine:
```
sudo apt-get install ntp
```
Then, add the following line in the file, where NTP-server-host is the hostname you specified for your NTP server:
```
server NTP-server-host prefer iburst
```

### Step 7: Restart the NTP server
In order for the above changes to take effect, you need to restart the NTP service. Run the following command as sudo in order to do so:
```
sudo service ntp restart
```
### Step 8: View the Time Synchronization Queue
Now your client and server machines are configured to be time-synced. You can view the time synchronization queue by running the following command:
```
ntpq -ps
```
You should be able to see NTP-server-host as the time synchronization host/source in the queue.

So this was all you needed to know about installing and configuring NTP to synchronize time on your networked Ubuntu machines. The process may seem a little cumbersome but if you follow all of the above steps carefully, one-by-one, your machines will be synced in no time.