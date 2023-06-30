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
* Usage:   VlCanApiTxRxExample [rtadcvhfm]
* Options: -h   Displays usage info.
*          -v   Verbose output for selected command line options. Default: Very little output.
*          -d <n>  Data bit rate (<= 5000000). Default: 5000000.
*          -a <n>  Arbitration bit rate (<= 1000000). Default: 1000000.
*          -c <n>  Number of test frames. Default: 100.
*          -r <1|0>  Receive port. Default: 0.
*          -t <1|0>  Transmit port. Default: 1.
*          -f <CAN_DATA_FILE_NAME> text file containing CAN packet information.
*          -m     Display the MCU firmware version information in xx.yy.zz format
* 
*
* To build the example application:
*     make -f VlCanApiTxRxExample.mk clean => Removes application and all dependant .o files.
*     make -f VlCanApiTxRxExample.mk all   => Builds VlCanApiTxRxExample application
*
* Build fruit is an example application, VlCanApiTxRxExample, in the current 
* directory.
* 
* Example output:
root@vl:nonAPICan# ./VlCanApiTxRxExample -a 1000000 -d 2000000 -c 20

VlCanApiTxRxExample version: 1.0
VlCanApiTxRxExample - Transmitting 20 packets out CAN port 0; Receiving on CAN port 1;
Bit Rates: Arbitration=1000000; Data=2000000;

CAN data file name: CANPacketsToTx.txt;
VersaAPI library version: 1.6.0
LIBUSB: 1.0.20.11004
Opening TX port 0 ...... OPENED;
Opening RX port 1 ...... OPENED;
MPEu-C1E FW version=0x010000;
---------------------------------------- 
SENDER_NUMBER_PKTS_SENT=20
RECEIVER_NUMBER_GOOD_PKTS_RECEIVED=20
RECEIVER_NUMBER_ERR_PKTS_RECEIVED=0
---------------------------------------- 





