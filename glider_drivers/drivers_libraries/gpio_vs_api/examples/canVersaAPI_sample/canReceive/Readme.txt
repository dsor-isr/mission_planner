* VersaLogic Example Code - 20191204
*
* This code is given with no guarantees and is intended to be used as an 
* example of how to access the VersaLogic MPEu-C1E CAN ports using VersaLogic's
* API VersaAPI. VersaAPI can be installed on a VersaLogic carrier board and
* non-VersaLogic carrier boards.
* 
* This code has been compiled and run using gcc version 5.4.0; Ubuntu
* 14.04.1 LTS (Xenial Xerus); kernel 4.4.0-148-generic; libusb version
* 1.0.20.11004.
*
* Application Description: 
* Opens the specified CAN port for Rx and waits to receive packets. When a
* packet is received it will print to the screen packet header information,
* include the first 4 bytes of data.
* The specified CAN port is opened with the following configuration:
*	- CAN FD packets
* 	- Bit-Rate-Switching enabled
*	- Loopback disabled 
*	- Receive enabled **
*	- Arbitration rate 1000000 **
*	- Data bit rate 2000000 **
*		** => Can be changed on the command line
* 
* Usage and Command Line Options: 
*    Usage:   can_rcv [acdhmv]
*    Options: 
*        -h      Displays usage info.
*        -a <n>  Arbitration bit rate (<= 1000000). Default: 1000000.
*        -c <n>  Number of test frames. Default: 100.
*        -d <n>  Data bit rate (<= 5000000). Default: 5000000.
*        -m      Display the MCU firmware version information in xx.yy.zz format
*        -v      Verbose output. Default 0 => very little output.
*
*
* To build the example application:
*     make -f can_rcv.mk clean => Removes application and all dependant .o files.
*     make -f can_rcv.mk all   => Builds can_rcv application
*
* Build fruit is an example application, can_rcv, in the current 
* directory.
* 
* Example output:
root@vl:nonAPICan# ./can_rcv -a 1000000 -d 2000000 -p 1

VersaLogic MPEu-C1E Packet Receive Example Application
Port Configuration (MBits/s):
        Port number:            1
        Arbitration bit rate:   1000000
        Data bit rate:          2000000

System Information:
        VersaAPI Version: 1.6.0
        LIBUSB Version: 1.0.20.11004

SUCCESS: Opened CAN Port 1 for receiving



