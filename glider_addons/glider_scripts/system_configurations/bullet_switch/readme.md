```

Tinkers: DSOR Group
         salmon      -> jcruz@tecnico.ulisboa.pt
         malaclemys  -> jquintas@gmail.com

Descripton: Documentation about switch_bullet
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

# Important Detail

This only works in pcs with parellel ports, example NANO-PV-D4251/N4551/D5251 (epic in mred, mblack, myellow). In the new pcs of the glider vehicle class this won't be necessary, because a GPIO port will be used instead, looking at you nvidia Nano, Xavier and TX's of this life.

# Parallel_port

Paralel_port is a c program to turn on external/internal bullets as well as mounting or unmounting an ssd card used in the glider vehicles. 

See [this](https://www.howtoforge.com/tutorial/accessing-parallel-ports-on-linux/) and [this](http://www6.uniovi.es/cscene/CS4/CS4-02.htmlfor) and [wikipedia](https://en.wikipedia.org/wiki/Parallel_port) for details on parallel ports.

## Installation

To install the paralel_port program in the vehicle use [install_bullet.bash](install_bullet.bash). 
This script will also compile the program and give the necessary permissions.

```bash
./install_bullet.bash
```

We stress out that in our configuration/installation of the vehicle this is done by the script *config_glider_os.sh*. 
```
@config_glider_os.sh
 . "${CATKIN_ROOT}/catkin_ws/src/glider_vx/glider_scripts/system_configurations/bullet_switch/install_bullet.bash"
```

Also through the gliderStartUp service we give the right permissions to the port:

```
@gliderStartUp
echo 'Giving leaks permission'
sudo chown 0.0 /home/glider/catkin_ws/devel/lib/leak_detector/leak_detector
sudo chmod ug+s /home/glider/catkin_ws/devel/lib/leak_detector/leak_detector
```

## Usage

To use the paralel_port program, the bash script [bullet](bullet), was created. Its usage is the following:

```bash
/etc/init.d/bullet {start|stop|restart|force-reload|external_ssd{on,off}|internal_ssd{on,off}}
```

* external_ssdon will turn on external and mount the ssd card
* external_ssdoff will turn on external and unmount the ssd card
* internal_ssdon will turn on internal and mount the ssd card
* internal_ssdoff will turn on internal and unmount the ssd card
* start, stop, restart and force-reload are the same as external_ssdoff


## How it works

Paralel_port can receive one or two arguments:
* The first can either be 'internal' or 'external'. Depending on this value, the flag **internal**, which is initialized with -1, will switch to 0 or 1.
* The second can either be 'ssdon' or 'ssdoff'. Depending on this value, the flag **ssd**, which is initialized with 0, will either remain 0 or switch to 1.

In the end of the program, the following line is called:

```c
outb(0xC8 | (internal?INTERNALPICO:EXTERNALPICO) | (ssd?SSDON:0x00), DATA);
```
where
```c
#define DATA	0x378
#define STATUS	(DATA+1)
#define INTERNALPICO	0x10
#define EXTERNALPICO	0x20
```
The following translates the meaning of the bits:
* bit 8 (Vcc Leak 2)
* bit 7 (Vcc Leak 1)
* bit 6 external pico
* bit 5 internal pico
* bit 4 Vcc picos
* bit 2 ssd

Since 0xC8 = 11001000, this line of code is writing to the address 0x378, *independently of the arguments*, 1 on bits 8,7 and 4.

NOTE: root permissions are necessary to access the parallel port.
This is checked with:
```c
if(ioperm(DATA, 1, 1)).
```



