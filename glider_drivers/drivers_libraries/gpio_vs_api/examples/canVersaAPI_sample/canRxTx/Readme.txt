* VersaLogic Example Code - 20210125
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
* packet is received it will print to the screen packet header information.
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
* Send/Receive packets out/in the specified port. Whether the port is sending or receving
* is determined by the -r and -t options. Each invocation of the sample application can either
* send or receive, one invocation cannot do both.

Usage:   rx_tx [rtadcvhfm]
Options: -h        Displays usage info.
         -v <n>    Verbose output for selected command line options. Default: Very little output.
         -d <n>    Data bit rate (<= 5000000). Default: 2000000.
         -a <n>    Arbitration bit rate (<= 1000000). Default: 1000000.
         -c <n>    Number of frames to send. Default: 100.
         -r <1|0>  Receive port. Default: 0.
         -t <1|0>  Transmit port. Default: 1.
         -m        Display the MCU firmware version information in xx.yy.zz format
         -p        Print the contents of the PSR register after every transmitted * 
         -f <CAN_DATA_FILE_NAME> Text file containing CAN packet information.
*
* To build the example application:
*     make -f rx_tx.mk clean => Removes application and all dependant .o files.
*     make -f rx_tx.mk all   => Builds rx_tx application
*
* Build fruit is an example application, rx_tx, in the current 
* directory.
* 
* Example output - Transmitting:
root@EPU-4012:/home/versaapi/can_110/examples/canVersaAPI_sample/canTxRx# ./rx_tx -t 0 -a 250000 -d 1000000 -c 22 -f ./try2.txt

rx_tx version: 1.1
Bit Rates: Arbitration=250000; Data=1000000;

CAN data file name: ./try2.txt;
VersaAPI library version: 1.1.0
LIBUSB: 1.0.21.11156

rx_tx - Transmitting 22 packets out CAN port 0;
MPEu-C1E FW version: 01.01.0d;

Opening TX port 0 ...... OPENED;
Packet #1:--->m s f n 0x000 0
Packet #2:--->m s f n 0x001 1 0x1
Packet #3:--->m s f b 0x002 2 0x1 0x2
Packet #4:--->m e s n 0x003 3 0x1 0x2 0x3
Packet #5:--->m e f n 0x004 4 0x1 0x2 0x3 0x4
Packet #6:--->m e f b 0x005 5 0x01 0x2 0x3 0x4 0x5
Packet #7:--->m s f b 0x006 6 0x1 0x2 0x3 0x4 0x5 0x6 
Packet #8:--->m e s n 0x007 7 0x1 0x2 0x3 0x4 0x5 0x6 0x7
Packet #9:--->m e f n 0x008 8 0x1 0x2 0x3 0x4 0x5 0x6 0x7 0x8
Packet #10:--->m s f b 0x079 0
Packet #11:--->m s f n 0x0A1 1 0x1
Packet #12:--->m s f b 0x0B0 2 0x1 0x2
Packet #13:--->m s f b 0x0B1 2 0x1 0x2
Packet #14:--->m e s n 0x0C1 3 0x1 0x2 0x3
Packet #15:--->m e f b 0x0D4 4 0x1 0x2 0x3 0x4
Packet #16:--->m e f b 0x0d3 4 0x1 0x2 0x3 0x4
Packet #17:--->m e f n 0x00E 5 0x01 0x2 0x3 0x4 0x5
Packet #18:--->m e f b 0x0F0 6 0x1 0x2 0x3 0x4 0x5 0x6 
Packet #19:--->m e f b 0x0F1 6 0x1 0x2 0x3 0x4 0x5 0x6 
Packet #20:--->m e s n 0x1BC 7 0x1 0x2 0x3 0x4 0x5 0x6 0x7
Packet #21:--->m e f b 0x1B1 8 0x1 0x2 0x3 0x4 0x5 0x6 0x7 0x8
Packet #22:--->m e f b 0x1BA 8 0x1 0x2 0x3 0x4 0x5 0x6 0x7 0x8
---------------------------------------- 
Number of CAN packets sent=22
Packet Error Counts:
ECR Register:
        Transmit Error Counter (TEC)=0
        Receive Error Counter  (REC)=0
        CAN Error Logging      (CEL)=0
 Receive Error Passive  (RP=0x0) is BELOW error level of 128
---------------------------------------- 
Closing TX port 0 ...... CLOSED;
No Rx port specified.



* Example output - Transmitting:
root@ben:/home/user/can_100/examples/canVersaAPI_sample/canTxRx# ./VlCanApiTxRxExample -r 0 -a 250000 -d 1000000

VlCanApiTxRxExample version: 1.0
Bit Rates: Arbitration=250000; Data=1000000;

CAN data file name: ;
VersaAPI library version: 1.1.0
LIBUSB: 1.0.21.11156

Setting up to receive packets on CAN port 0;
MPEu-C1E FW version: 01.01.0d;

No Tx port specified.
Opening RX port 0 ...... OPENED;

Waiting to receive CAN packets on port 0 ...
Command:RX; Size:16; Port:0
        Id:             0x000
        SIF:            Data
        ESI:            Active
        Data Length:    0
        BRS:            Off
        Format:         CAN FD
        Time Stamp:     54722
        Filter Index:   63
        Frame Match:    1

Command:RX; Size:17; Port:0
        Id:             0x001
        SIF:            Data
        ESI:            Active
        Data Length:    1
        BRS:            Off
        Format:         CAN FD
        Time Stamp:     2376
        Filter Index:   63
        Frame Match:    1
        Data Dump:      01 
...
...
...

Command:RX; Size:23; Port:0
        Ext Id:         0x000001BC
        SIF:            Data
        ESI:            Active
        Data Length:    7
        Format:         CAN
        Time Stamp:     24544
        Filter Index:   63
        Frame Match:    1
        Data Dump:      01 02 03 04 05 06 07 

Command:RX; Size:24; Port:0
        Ext Id:         0x000001BA
        SIF:            Data
        ESI:            Active
        Data Length:    8
        BRS:            On
        Format:         CAN FD
        Time Stamp:     62498
        Filter Index:   63
        Frame Match:    1
        Data Dump:      01 02 03 04 05 06 07 08 

:q
SUCCESS: Closed CAN Port;
---------------------------------------- 
Number of good CAN packets received=22
---------------------------------------- 


