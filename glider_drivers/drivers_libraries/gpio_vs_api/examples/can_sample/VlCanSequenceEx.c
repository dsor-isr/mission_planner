/* VersaLogic CAN FD Transmit example
*
* This code is given with no guarantees and is intended to be used as an 
* example of how to use the Linux native software packages to access
* the VersaLogic MPEu-C1E mPCIe CAN card.
*
* Default Program Description: 
* Transmit a set number of CAN FD packets with payload of '0' '1' '0' '1' ... 
* out both ports.
* 
* Usage:   vlCanSequenceEx [acdhmrv]\n" );
*    Options:	-h   Displays usage information
*    			-v   Verbose output for selected command line options. Default: 
*					 Very little output
*    			-d <n>  Data bit rate (<= 5000000). Default: 5000000
*    			-a <n>  Arbitration bit rate (<= 1000000). Default: 1000000
*    			-c <n>  Number of test frames. Default: 100
*    			-m 	Display the MCU firmware version information in 
*					xx.yy.zz format
*			-r	Enable RX on TX
* 
* This code has been compiled and run using gcc version 5.4.0 on Ubuntu 
* 16.04.01 LTS, kernel 4.4.0-148-generic, libusb version 1.0.20.11004. 
* A compatible Makefile has been provided with this .c file for an example 
* of how to compile this code. 

Revision	1.0: -	Initial version.
			1.1: -	Added the -m command line option to just display the MCU
					version string.

* Copyright 2019 VersaLogic Corporation
*/


/* ***** Includes Files. ***** */
// System.
#include <ctype.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// VersaAPI header.
#include "VL_CAN.h"
/* *************************** */

/* ***** Defines. ***** */
#define TX_BUF_SIZE         ( 64 )		// Transmit data buffer size.
#define BITRATE_ARBITRATION (1000000)	// Default abitration bit rate.
#define BITRATE_DATA        (5000000)	// Default data bit rate.
#define NUM_FRAMES          (100)		// Default number of frames to transmit.
/* ******************** */


/* ***** Global variables. ***** */
MCAN_TX_PACKET	gcurPacket;						// CAN Packet to send.
uint8_t	gOutBuf[TX_BUF_SIZE];					// CAN packet data.
int 		gCurPktCount0		= 0;			// Number of packets sent Port0.
int 		gCurPktCount1		= 0;			// Number of packets sent Port1.
int 		gCurRxPktCount		= 0;			// Current number of packets received.
int			gFailedPktCount0	= 0;			// Assume all is ok.
int			gFailedPktCount1	= 0;			// Assume all is ok.
int         gTxPortNum0			= 0;			// Port 0.
int         gTxPortNum1			= 1;			// Port 1.
int         gUseUserTiming      = 0;            // 0 (default) Use default values; 
                                                // 1 => Use user timings values.

// These variables can be overridden on the command line.
int			gCmdVerboseFlag		= 0;					// Verbosity off.
int			gNumTestFrames		= NUM_FRAMES;			// Number of packets to send.
uint32_t    gBitRateArb			= BITRATE_ARBITRATION;	// Arbitration bit rate.
uint32_t    gBitRateData		= BITRATE_DATA;			// Data bit rate.
uint32_t	gRxOnTx				= 1;					// Allow for RX on a TX.
														// NOTE 20191203: Setting default
														// to 1 because of a getMCUVersion
														// issue needing the RX to be on.

int	 gAPIDebugLevel				= 0;					// No debug messages.
bool gAbortFlag					= false;
/* ***************************** */


/* *************************************************************************
   Function that creates a CAN packet and trasmits it out both
   CAN Ports.
   ************************************************************************* */
void sendAllPackets()
{
	/* ***** Declarations. ***** */
	CmdStatusT	returnCode;
	/* ************************* */

	/* ***** Where are we. ***** */
	if (gAPIDebugLevel >= DEBUG_1)
	{
		printf("Entering sendAllPackets()\n");
	}
	/* ************************* */

	/* ***** Initializations. ***** */
	gFailedPktCount0	= 0;
	gFailedPktCount1	= 0;
	gCurPktCount0		= 0;
	gCurPktCount1		= 0;
	/* **************************** */

	/* ***** Construct the packet. ***** */
	// Create an extended frame packet as worste case.
	// Set ID to 0.
	gcurPacket.t0 = 0;

	// Use CAN FD, 64 data byte packet, Bit Rate Swtiching.
	gcurPacket.t1 |= CAN_RX_TX_CAN_FD_FORMAT;
	gcurPacket.t1 |= CAN_RX_TX_BR_SWITCHING;

	// Put the CAN length code into t1.
    gcurPacket.t1 &= ~CAN_RX_TX_SIZE_64;
    gcurPacket.t1 |= (0xF << 16);

	// Put data into the tx buffer.
	for (int i = 0; i <= 64; i++)
	{
		gcurPacket.mcanTxData[i] = i%2;
	}
	/* ********************************* */

	/* ***** Send the packets out until count is reached. ***** */
	printf("Sending packets ");
	while ((gCurPktCount0 + gFailedPktCount0) < gNumTestFrames)
	{
			printf(".");
			// Send the packet out Port0.
			returnCode = ex_CanTransmit(VL_CAN_BOARD0, VL_CAN_PORT0, &gcurPacket);
			if (returnCode == VL_API_OK)
			{
				gCurPktCount0++;
			}
			else
			{
				printf("Transmit out Port 0 FAILED\n");
				gFailedPktCount0++;
			}

			// Send the packet out Port1.
			returnCode = ex_CanTransmit(VL_CAN_BOARD0, VL_CAN_PORT1, &gcurPacket);
			if (returnCode == VL_API_OK)
			{
				gCurPktCount1++;
			}
			else
			{
				printf("Transmit out Port 1 FAILED\n");
				gFailedPktCount1++;
			}

		// For last frame sent set TX event bit to get an TX ack.
    	if ((gCurPktCount0 + gFailedPktCount0) == (gNumTestFrames - 1) )
    	{
			gcurPacket.t1 |= CAN_RX_TX_CAN_TX_EVENT;
		}
	}  // while(gCurPktCount ...)
	printf(" Done\n");
	/* ******************************************************** */

	/* ***** Print transmit results. ***** */
	printf("\n---------------------------------------- \n");
	printf("Number of packets SUCCESSFULLY sent out Port%d:\t%d\n", gTxPortNum0, gCurPktCount0);
	printf("Number of packets SUCCESSFULLY sent out Port%d:\t%d\n", gTxPortNum1, gCurPktCount1);
	printf("Number of packets attempted to send but FAILED:\t%d\n", gFailedPktCount0);
	printf("Number of packets attempted to send but FAILED:\t%d\n", gFailedPktCount1);
	printf("Total Bytes attempted to send:\t%11ld\n", (long)((gNumTestFrames * 2) * 64));
	printf("Total Bytes SUCCESSULLY sent:\t%11ld\n", (long)((gCurPktCount0 + gCurPktCount1) * 64));
	printf("Total Bytes FAILED to send:\t%11ld\n", (long)((gFailedPktCount0 + gFailedPktCount1) * 64));
	printf("---------------------------------------- \n");
	/* ********************************** */


	/* ***** Done. Clean up. ***** */
	if (gAPIDebugLevel >= DEBUG_1)
		printf("Leaving sendAllPackets()\n");

	return;
	/* *************************** */
}


/* ***** Function to display application help message. ***** */
void showUsage()
{
	printf( "\nUsage:   VlCanSequenceEx [acdhmv]\n" );
    printf( "Options: -h   Displays usage info.\n" );
    printf( "         -v   Verbose output for selected command line options. Default: Very little output.\n" );
    printf( "         -d <n>  Data bit rate (<= 5000000). Default: 5000000.\n" );
    printf( "         -a <n>  Arbitration bit rate (<= 1000000). Default: 1000000.\n" );
    printf( "         -c <n>  Number of test frames. Default: 100.\n" );
    printf( "         -m 	Display the MCU firmware version information in xx.yy.zz format\n" );
    printf( "         -r 	Allow for receiving a packet back after it is sent. Value 0|1. Default is 0 (no).\n" );
    printf( "         -u 	Set CANOpen() call to use: Default is to use ex_CanOpen(); Use ex_CanOpenWithUserTiming().\n");
    printf( "\n" );

	exit(-1);
}
/* ********************************************************* */

/* ********************************************* */
/* ***** Function to print error messages. ***** */
/* ********************************************* */
void printErrorCode(CmdStatusT errorCode)
{
	switch(errorCode)
	{
		case VL_API_OK:							printf("VL_API_OK\n"); break;
		case VL_API_ERROR:						printf("VL_API_ERROR\n"); break;
		case VL_API_INVALID_ARG:				printf("VL_API_INVALID_ARG\n");  break;
		case VL_API_NOT_SUPPORTED:				printf("VL_API_INVALID_ARG\n");  break;
		case VL_API_CAN_CLOSE_DEVICE_ERROR:		printf("VL_API_CAN_CLOSE_DEVICE_ERROR\n"); break;
		case VL_API_CAN_OPEN_DEVICE_ERROR: 		printf("VL_API_CAN_OPEN_DEVICE_ERROR\n"); break;
		case VL_API_CAN_OPEN_NO_DEVICE_FOUND: 	printf("VL_API_CAN_OPEN_NO_DEVICE_FOUND\n"); break;
		case VL_API_CAN_OPEN_NO_DEVICE_DESC: 	printf("VL_API_CAN_OPEN_NO_DEVICE_DESC\n"); break;
		case VL_API_CAN_OPEN_PORT_ERROR:		printf("VL_API_CAN_OPEN_PORT_ERROR\n"); break;
		case VL_API_CAN_PORT_CLOSED:			printf("VL_API_CAN_PORT_CLOSED_ERROR\n"); break;
		case VL_API_CAN_TX_ERROR:		 		printf("VL_API_CAN_TX_ERROR\n"); break;
		default:								printf("VL_UNKNOWN_ERROR_CODE\n");
	}
}

/*****************************************************************************/
/***  signalHandler - CTRL-C   ***********************************************/
/*****************************************************************************/
static void signalHandler(int signalNum)
{
    gAbortFlag = true;

    return;
}
/* ********************************************* */


/* ************************************************************************ */
/* Function to retrieve and display the MCU firmware version number.        */
/* ************************************************************************ */
static void displayMCUVersionNumber()
{
	/* ***** Declarations. ***** */
	unsigned long vl_ReturnStatus;	// Return status.
	uint32_t fwVer;
	/* ************************* */

	/* ***** Initializations. ***** */
	vl_ReturnStatus	= VL_API_OK;
	fwVer			= (uint32_t)0;
	/* **************************** */

	/* ***** Retrieve the MCU firmware version. ***** */
	// Need to open a channel to the MCU.
	vl_ReturnStatus = ex_CanOpenPort(VL_CAN_BOARD0, gTxPortNum0, gBitRateArb, 
									 gBitRateData, VL_ENABLE_RX_ON_TX, VL_DISABLE_LOOPBACK);

	if (vl_ReturnStatus != VL_API_OK)
	{
		printf("Retrieving MCU FW version: ERROR: ");
		printErrorCode(vl_ReturnStatus);
		printf("\n");
	}

	// Returning a value just in case we need it later.
	vl_ReturnStatus = ex_CanGetMCUFwVersion(0, &fwVer);
	printf("0x%x\n", fwVer);
	printf("%02x.%02x.%02x;\n", (fwVer >> 16), ((fwVer >> 8) & 0xF), (fwVer & 0xF));

	// Return value here does not really matter.
	vl_ReturnStatus = ex_CanClosePort(VL_CAN_BOARD0, gTxPortNum0);
	/* ********************************************** */

	/* ***** Done. ***** */
	return;
	/* ***************** */
}


/* ***** main program. ***** */
int main(int argc, char *argv[])
{
	/* ***** Types. ***** */
	/* ****************** */

    /* ***** Variable declarations. ***** */
    unsigned long 	vl_ReturnStatus;	// Return status.
    int			  	opt;				// Command line option helper.
	uint32_t 		fwVer;				// MCU FW version.
	struct sigaction	sigact;			// CTRL-C processing.
    /* ********************************** */

    /* ***** Variable initializations. ***** */
    vl_ReturnStatus = 0xffffff;	// Assume a failure on open.
	opt				= 0;  		// Process command line options.
    /* ************************************* */

    /* ***** Process command line options. ***** */
    while ((opt = getopt(argc, argv, "rmf:c:v:a:d:uh")) != -1)
    {
		switch(opt)
		{
			case 'c':	gNumTestFrames = atoi(optarg);
						break;
			case 'v':  // Verbose mode.
						gAPIDebugLevel = atoi(optarg);
						break;
			case 'r':  // Enable RX on TX.
						gRxOnTx = 1;
						break;
			case 'a': // Arbitration bit rate.
						gBitRateArb = atoi(optarg);
						break;
			case 'd': // Data bit rate.
						gBitRateData = atoi(optarg);
						break;
			case 'm': // Display the MCU version number in MM.mm.rr format.
						displayMCUVersionNumber();
						exit(1);
			case 'u': // Data bit rate.
						gUseUserTiming = 1;
						break;
			case 'h':
			default:	showUsage();
						exit(-1);
		}
	}
    /* ***************************************** */

	/* ***** Setup CTRL-C processing. ***** */
    sigact.sa_handler = signalHandler;
    sigemptyset( &sigact.sa_mask );
    sigact.sa_flags = 0;
    sigaction( SIGINT, &sigact, NULL );
    sigaction( SIGTERM, &sigact, NULL );
    sigaction( SIGQUIT, &sigact, NULL );
	/* ************************************ */

	/* ***** Print example information. ***** */
	const struct libusb_version *pLUVersion;
    pLUVersion = libusb_get_version();
	printf("\nVersaLogic MPEu-C1E CAN FD Transmit Example\n");
	printf("\tTransmitting %d packets with payload of '0' '1' '0' '1' ... out both ports.\n", 
		gNumTestFrames);

	printf("Port Configuration (MBits/s):\n");
	printf("\tArbitration bit rate:\t%d\n\tData bit rate:\t\t%d\n\n", 
		gBitRateArb, gBitRateData);

	// libusb version.
    printf("\tLIBUSB Version: %s%d.%d.%d.%d\n\n", 
		pLUVersion->rc, pLUVersion->major, pLUVersion->minor, 
		pLUVersion->micro, pLUVersion->nano );
	/* ************************************** */


	/* ***** Make sure all options are set that need to be set. ***** */
	if ((gBitRateArb < 0) || (gBitRateArb > 1000000))
	{
		printf("Arbitration bit rate must be > 0 and < 1000000;\n");
		showUsage();
		exit(-1);
	}

	if ((gBitRateData < 0) || (gBitRateData > 5000000))
	{
		printf("Data bit rate must be > 0 and < 5000000;\n");
		showUsage();
		exit(-1);
	}
	/* ************************************************************** */

	/* ***** Open both ports. ***** */
	// Transmit port 0. 
	printf("Opening TX port %d ...", gTxPortNum0);
	if (0 == gUseUserTiming)
	{
	    vl_ReturnStatus = ex_CanOpenPort(VL_CAN_BOARD0, gTxPortNum0, gBitRateArb, 
									  gBitRateData, gRxOnTx, VL_DISABLE_LOOPBACK);
	}
	else
	{
	    vl_ReturnStatus = ex_CanOpenPortWithUserTiming(VL_CAN_BOARD0, 0,         
					              (uint32_t)1000000,            // Arbitration rate
								  (uint32_t)2000000,            // Data rate
								  (uint8_t)VL_ENABLE_RX_ON_TX, 
								  (uint8_t)VL_DISABLE_LOOPBACK,
								  (uint32_t)5,                                      // Master divider
								  (uint16_t)1, (uint8_t)3, (uint8_t)11, (uint8_t)4,  // arbitration timing.
								  (uint16_t)0, (uint8_t)3, (uint8_t)11, (uint8_t)4   // data timing.
								  );
	}


	if (vl_ReturnStatus != VL_API_OK)
	{
		printf("... ERROR:");
		printErrorCode(vl_ReturnStatus);
		printf("\n");
	}
	else
	{
		printf("... SUCCESS\n");
	}

	// Transmit port 1. 
	printf("Opening TX port %d ...", gTxPortNum1);
	if (0 == gUseUserTiming)
	{
	    vl_ReturnStatus = ex_CanOpenPort(VL_CAN_BOARD0, gTxPortNum1, gBitRateArb, 
									  gBitRateData, gRxOnTx, VL_DISABLE_LOOPBACK);
	}
	else
	{
	    vl_ReturnStatus = ex_CanOpenPortWithUserTiming(VL_CAN_BOARD0, 1,         
					              (uint32_t)1000000,            // Arbitration rate
								  (uint32_t)2000000,            // Data rate
								  (uint8_t)VL_ENABLE_RX_ON_TX, 
								  (uint8_t)VL_DISABLE_LOOPBACK,
								  (uint32_t)5,                                      // Master divider
								  (uint16_t)1, (uint8_t)3, (uint8_t)11, (uint8_t)4,  // arbitration timing.
								  (uint16_t)0, (uint8_t)3, (uint8_t)11, (uint8_t)4   // data timing.
								  );
	}

	if (vl_ReturnStatus != VL_API_OK)
	{
		printf("... ERROR:");
		printErrorCode(vl_ReturnStatus);
		printf("\n");
	}
	else
	{
		printf("... SUCCESS\n");
	}
	/* **************************** */

	/* ***** Print version number of the MCU code. ***** */
	fwVer = 0;

	// Get CAN card version information.
	vl_ReturnStatus = ex_CanGetMCUFwVersion(0, &fwVer);
	printf("MPEu-C1E FW Version:0x%06x;\n\n", fwVer);
	/* ************************************************* */

	/* ***** Send the desired number of packets. ***** */
	// Send the data.
	sendAllPackets();
	/* *********************************************** */

	/* ***** Done. ***** */
	vl_ReturnStatus = ex_CanClosePort(VL_CAN_BOARD0, gTxPortNum0);
    return(0);
	/* ***************** */
}

