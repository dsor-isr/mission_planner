* VersaLogic Example Code - 20210125
*
* This code is given with no guarantees and is intended to be used as an 
* example of how to access the VersaLogic MPEu-C1E CAN ports using only
* standard Linux libraries.
* 
* This code has been compiled and run using gcc version 7.5.0; Ubuntu
* 18.04.5 LTS (Xenial Xerus); kernel 4.15.0-134-generic; libusb version
* 1.0.21.11156.
*
* Application Description: 
* Transmit on both ports a number of CAN FD packets with payload of 
* '0' '1' '0' '1' ... .
* Both CAN ports are opened with the following configuration:
*	- CAN FD packets
* 	- Bit-Rate-Switching enabled
*	- Loopback disabled 
*	- Receive enabled **
*	- Arbitration rate 1000000 **
*	- Data bit rate 2000000 **
*		** => Can be changed on the command line
* 
* Usage and Command Line Options: 
*    Usage:   vlCanSequenceEx [acdhmrv]
*    Options: 
*        -h      Displays usage info.
*        -a <n>  Arbitration bit rate (<= 1000000). Default: 1000000.
*        -c <n>  Number of test frames. Default: 100.
*        -d <n>  Data bit rate (<= 5000000). Default: 5000000.
*        -m      Display the MCU firmware version information in xx.yy.zz format
*        -r      Allow for receiving a packet back after it is sent. Value 0|1. 
*				 Default is 1 (on).
*        -v      Verbose output. Default 0 => very little output.
*        -u      Use ex_CanOpenPortWithUserTiming() instead of ex_CanOpenPort().
*
*
* To build the example application:
*     make -f VlCanSequenceEx.mk clean => Removes application and all dependant .o files.
*     make -f VlCanSequenceEx.mk all   => Builds vlCanSequenceTest application
*
* Build fruit is an example application, vlCanSequenceTest, in the current 
* directory.
* 
* Example output:
root@vl:nonAPICan# ./vlCanSequenceTest -c 10 -a 1000000 -d 2000000

VersaLogic MPEu-C1E CAN FD Transmit Example
        Transmitting 10 packets with payload of '0' '1' '0' '1' ... out both ports.
Port Configuration (MBits/s):
        Arbitration bit rate:   1000000
        Data bit rate:          2000000

        LIBUSB Version: 1.0.20.11004

Opening TX port 0 ...... SUCCESS
Opening TX port 1 ...... SUCCESS
MPEu-C1E FW Version:0x010000;

Sending packets .......... Done

---------------------------------------- 
Number of packets SUCCESSFULLY sent out Port0:  10
Number of packets SUCCESSFULLY sent out Port1:  10
Number of packets attempted to send but FAILED: 0
Number of packets attempted to send but FAILED: 0
Total Bytes attempted to send:         1280
Total Bytes SUCCESSULLY sent:          1280
Total Bytes FAILED to send:               0
---------------------------------------- 

*

