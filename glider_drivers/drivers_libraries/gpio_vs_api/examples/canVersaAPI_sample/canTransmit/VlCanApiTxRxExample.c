/* VersaLogic VlCanApiTxRxExample sample application
*
* This code is given with no guarantees and is intended to be used as an 
* example of how to use the VersaAPI software package and to check the 
* operation post install.
*
* Default Program Description: 
* Requires that the two CAN ports be connected to each other. One CAN port
* will be the receive port (-r) the other CAN port will be the transmit (-t) port.
* Packets will be read out of a text file (see example packet file), trasmitted
* out the tx port and received on the rx port. A count of the number of packets
* tranmitted, the number of packets received and the number of packets that
* timeout is reported at the end of the test. The number of packets transmitted
* is set by the -c command line option.
* 
* Command Line Options: 
* 
* This code has been compiled and run using gcc on Ubuntu 16.04.01 LTS, 
* kernel x.y.z-aa-generic, with VersaAPI version 1.6.0.  A Linux
* compatible Makefile has been provided with this .c file for an example
* of how to compile this code. 

Revision	1.0	20191219:	-Initial version.
*/


/* ***** Includes Files. ***** */
// System.
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>

// VersaAPI header.
#include "VL_OSALib.h"
/* *************************** */

/* ***** Defines. ***** */
#define VER_STRING "1.0"	// Version string of this application.
#define SLEEP_TIME 3 		// Time to sleep between setting the DIO.
#define LOOP_COUNT 3		// Number of times to toggle the DIO.
#define VL_RETURN_SUCCESS 0	// A successful return status from a VL API call.

#define PCR_REGISTER 0x00	// FPGA offset of the PCR register.
#define AUXMODE2 0x2B		// FPGA offset of the AUXMODE2 register.

#define YES (1)				// Define yes.
#define NO  (0)				// Define no.
#define false (0)
#define true  (1)

#define SIG_VL_GPIO 56		// VersaLogic 8256 Signal identifier.
#define TX_BUF_SIZE         ( 64 )

#define MAX_STD_ID          (0x7FF)
#define MAX_EXT_ID          (0x1FFFFFFF)

#define BITRATE_ARBITRATION (1000000)
#define BITRATE_DATA        (5000000)
#define NUM_FRAMES          (100)

// Helpful print strings.
static char *pCmdStr[] = {
    "UNDEFINED",
    "OPEN",
    "CLOSE",
    "TX",
    "RX",
    "STS",
    "ID",
    "PROGRAM",
	"RX_CONTROL",
	"MCU_FW_VER"
};

static char *pStsStr[] = {
    "OK",
    "ERROR",
    "TX BUSY",
    "TX ERROR",
    "RATE ERROR",
    "INVALID PORT"
};


/* ******************** */


/* ***** Global variables. ***** */
int TimerTriggered;			// Global variable: Initial condition = NO,
							// After timer has triggered = YES.

char	*pCANPacketFileName	= "./CANPacketsToTx.txt";
FILE	*pCANPacketFile		= (FILE *)NULL;
MCAN_TX_PACKET	curPacket;

uint8_t	gOutBuf[TX_BUF_SIZE];
char	gInputFileName[128];

int 		gCurPktCount	= 0;			// Current number of packets sent.
int 		gCurRxPktCount	= 0;			// Current number of packets received.
int			gCurTxRxPktCount	= 0;		// Number of Tx'ed packets echo back.
											// Controlled by OpenPort API call.
int			gNumTestFrames	= NUM_FRAMES;	// Number of packets to send.
int			gFailedPktCount	= 0;			// Assume all is ok.
int			gTimeOutPktCount= 0;			// Number of packets that have timed out.
int         gTxPortNum		= 1;			// 0, 1 - Valid for ports. Default 1.
int         gRxPortNum		= 0;			// 0, 1 - Valid for ports. Default 0.
int			gCmdVerboseFlag	= 0;				// Verbosity off.
bool        gAbortFlag 		= false;
uint32_t    gBitRateArb		= BITRATE_ARBITRATION;
uint32_t    gBitRateData	= BITRATE_DATA;
/* ***************************** */

/*****************************************************************************/
/***   dumpMem   *************************************************************/
/*****************************************************************************/
/*
* DUMP FORMAT:
*             1111111111222222222233333333334444444444555555555566666666667777
*   01234567890123456789012345678901234567890123456789012345678901234567890123
*   AAAA: XX XX XX XX XX XX XX XX  XX XX XX XX XX XX XX XX  >0123456789ABCDEF<
*/
#define DUMP_ADDR       (0)
#define DUMP_HEX_DATA   (6)
#define DUMP_ASCII_GT   (56)
#define DUMP_ASCII_DATA (57)
#define DUMP_ASCII_LT   (73)
#define DUMP_STR_LENGTH (80)
#define MCAN_NUM_LENGTHS        (16)

int gMcanSizeTable[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8,
    12, 16, 20, 24,
    32, 48, 64
};

uint32_t gMcanLengthCodes[] = {
    0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8,
    0x9, 0xA, 0xB, 0xC,
    0xD, 0xE, 0xF
};


int mcan_length_to_size(int lengthCode)
{
    int         i;
    int         size;

    size = 0xFF;
    for ( i = 0; i < MCAN_NUM_LENGTHS; i++ )
    {
        if ( lengthCode == gMcanLengthCodes[ i ] )
        {
            size = gMcanSizeTable[ i ];
            break;
        }
    }

    return( size );
}


void dumpMem(FILE *outFile, uint8_t* pData, int length)
{
    int  i 		= 0;
    int  addr	= 0;

    while (i < length)
    {
        /* Add address to output string */
        if ( 0 == i )
        {
			printf("0x%04x: ", addr);
        }

		printf("0x%02x ", pData[i]);

		// Increment counters.
		i ++;
		addr ++;

    }   // while (i < length )

    return;
}


/* ***** getIdLength() ***** */
/* ************************* */
size_t getIdLength(char *pStr)
{
	size_t length = 0;

	/* ***** Where are we. ***** */
	if (gAPIDebugLevel >= VL_DEBUG_1)
		printf("Entering getIdLength(pStr=%s)\n", pStr);
	/* ************************* */

	printf("----->*pStr=%c<-----\n", *pStr);
	while ((*pStr != ' ') || (*pStr != '\t') || (*pStr != '\n') || 
		   (*pStr != '#'))
	{
		length ++;
		pStr ++;
		printf("----->*pStr=%c<-----\n", *pStr);
	}

	/* ***** Done. ***** */
	if (gAPIDebugLevel >= VL_DEBUG_1)
		printf("Leaving getIdLength()\n");

	return(length);
	/* ***************** */
}


/* ****************************************************************************
//	mcan_prt_vl_pkt()
//	Print a CAN packet.
//	
//	Inputs: 
//		*pPacket	=> Pointer to the desired CAN packet.
//
//	Outputs:
//		None.
//
//	Return Value:
//		None (void).
   ************************************************************************** */
void mcan_prt_vl_pkt(VL_USB_CAN_XFER     *pPacket)
{
    int                 frameSize;

    // Need to keep updated with the number of available commands.
    if ( pPacket->cmd >= 0 && pPacket->cmd <= USB_CAN_CMD_MAX )
    {
        printf(
          "Command: %s  Size: %d Port: %d\n",
		  pCmdStr[ pPacket->cmd ],
          (int) pPacket->size,
          (int) pPacket->port
        );
    }

    switch ( pPacket->cmd )
    {
    case USB_CAN_CMD_UNDEFINED:
    case USB_CAN_CMD_CLOSE:
    case USB_CAN_CMD_ID:
    //case USB_CAN_CMD_PROGRAM:
        printf( "\n" );
        break;

    case USB_CAN_CMD_OPEN:
        printf( "    arb rate: %ld\n", (long) pPacket->u.openInfo.arbitrationRate );
        printf( "    data rate: %ld\n", (long) pPacket->u.openInfo.dataRate );
        printf( "    loop back: %d\n\n", pPacket->u.openInfo.loopback ? 1 : 0 );
        break;

    case USB_CAN_CMD_TX:
    case USB_CAN_CMD_RX:
        // T0 and R0
        if ( pPacket->u.txPacket.t0 & CAN_RX_TX_EXTENDED_ID )
        {
            printf( "       ext id: 0x%08X\n", CAN_RX_TX_GET_EXID( pPacket->u.txPacket.t0 ) );
        } else {
            printf( "           id: 0x%03X\n", CAN_RX_TX_GET_ID( pPacket->u.txPacket.t0 ) );
        }
        printf( "        frame: %s\n", pPacket->u.txPacket.t0 & CAN_RX_TX_REMOTE_FRAME ? "remote" : "standard" );
        printf( "        error: %s\n", pPacket->u.txPacket.t0 & CAN_RX_TX_ERR_PASSIVE ? "passive" : "active" );

        // T1 and R1
        frameSize = mcan_length_to_size( (pPacket->u.txPacket.t1 >> 16) & 0xF );
        printf( "   frame size: %d\n", frameSize );
        printf( "  rate switch: %d\n", pPacket->u.txPacket.t1 & CAN_RX_TX_BR_SWITCHING ? 1 : 0 );
        printf( "       format: %s\n", pPacket->u.txPacket.t1 & CAN_RX_TX_CAN_FD_FORMAT ? "CAN FD" : "CAN" );

        if ( USB_CAN_CMD_RX == pPacket->cmd )
        {
            printf( "   time stamp: %d\n", pPacket->u.rxPacket.r1 & 0xFFFF );
            printf( " filter index: %d\n", (pPacket->u.rxPacket.r1 >> 26) & 0x7F );
            printf( "  frame match: %d\n", (pPacket->u.rxPacket.r1 & 0x80000000) ? 1 : 0 );
        } else {
            printf( " store events: %s\n", (pPacket->u.txPacket.t1 & 0x00800000) ? "yes" : "no" );
            printf( "       marker: 0x%02X\n", (pPacket->u.txPacket.t1 & 0xFF000000) >> 24 );
        }

        // If there is data do a hex dump of it.
		frameSize = (int)pPacket->size - 12;
        if ( frameSize )
        {
            // NOTE: mcanTxData and mcanRxData will be the same.
            dumpMem(stdout, (uint8_t*) &(pPacket->u.rxPacket.mcanRxData), frameSize);
        }

        printf( "\n" );

        break;

    case USB_CAN_CMD_STS:
        if ( CAN_STS_UNDEFINED == (pPacket->misc & ~CAN_STS_OPENED) )
        {
            printf(
              "          sts: UNDEFINED - %s\n\n",
              (pPacket->misc & CAN_STS_OPENED) ? "OPEN" : "CLOSE" 
            );
        } else {
            printf( "          sts: %s - %s\n\n",
              pStsStr[ pPacket->misc & ~CAN_STS_OPENED ],
              (pPacket->misc & CAN_STS_OPENED) ? "OPEN" : "CLOSE" 
            );
        }
        break;

    default:
        printf(
          "ERR: Unknown command 0x%02d size %d port %d\n\n",
          (int) pPacket->cmd,
          (int) pPacket->size,
          (int) pPacket->port
        );

        break;
    }

    return;
}


// User Defined function called when a packet is recieved.
void user_CANRxCallBackFunction(VL_USB_CAN_XFER *pRxPacket, int txStatus)
{
	/* ***** Declarations. ***** */
	/* ************************* */
	
	/* ***** Initializations. ***** */
	/* **************************** */

	/* ***** PROCESS. ***** */
	if (gAPIDebugLevel >= VL_DEBUG_1)
	{
		printf("Entering app userDefinedRxFunction(), txStatus=%d\n", txStatus);
	}	

	if(txStatus == LIBUSB_TRANSFER_COMPLETED)
	{
		if(pRxPacket->cmd == USB_CAN_CMD_RX)
		{
			gCurRxPktCount ++;
			gCurTxRxPktCount ++;
		}
		
		if (gAPIDebugLevel >= VL_DEBUG_1)
		{
			//mcan_prt_vl_pkt_header("app userDefinedRxFunction: ", pRxPacket);
			VSL_CANPrintPktHeader("app userDefinedRxFunction: ", pRxPacket);
			mcan_prt_vl_pkt(pRxPacket);
		}
	}
	if (txStatus == LIBUSB_TRANSFER_TIMED_OUT)
	{
		gTimeOutPktCount++;
	}
	else
	{
		gFailedPktCount ++;
	}
	/* ******************** */

    if (gAPIDebugLevel >= VL_DEBUG_1)
		printf("Leaving app userDefinedRxFunction()\n");

	return;
}


/* ***** Function that reads a file containing CAN packets and transmits *****
   ***** out the specified port.                                         ***** */
/* ***** File format of the CAN Input file. *****
   Format is consistent with Peak Systems test program for driver 8.6.0. Any
   incompatibilities will be explicitly marked in the code. 
	#			=> comment, ignored.
	m|r			=> m = normal message; r = rtr message
	s|e			=> s = standard; e = extended
	msg_id		=> message ID. < 0x3fffffff or < 2047 
	data_len	=> amount of data, in bytes.  If specified data is < data_len
				   the data area will be populated with the last byte until
				   data_len is reached. If amount of data > data_len, the 
				   data will be truncated at data_len.

   Examples:
	# Standard messages
	m s 0x7FF 0 # a comment
	m s 0x000 1 0x11
	m s 0x7FE 2 0x22 0x33

	# Extended messages
	m e 0x00000000 1 0x11
	m e 0x00000001 3 0x33 0x44 0x55

	# Remote standard messages
	r s 0x005 2 0x22 0x33
	r s 0x006 4 0x44 0x55 0x66 0x77
   ********************************************** */

/* ***** sendAllPackets(Board, Port) ***** */
void sendAllPackets()
{
	/* ***** Declarations. ***** */
	char 	dataRead[1024];
	uint8_t tmpInt;
	uint32_t	extId;

	VL_APIStatusT	returnCode;
	char 			*curToken;
	int	 			extendedId;
	int				canFDEnabled;
	uint32_t		canLengthCode;
	/* ************************* */

	/* ***** Where are we. ***** */
	if (gAPIDebugLevel >= VL_DEBUG_1)
		printf("Entering sendAllPackets()\n");
	/* ************************* */

	/* ***** Initializations. ***** */
	gFailedPktCount	= 0;
	gCurPktCount	= 0;
	/* **************************** */

	/* ***** Process each packet line in the input file. ***** */
	// Not at the end of the file. Get the next packet.
	//while (fgets(dataRead, 1024, pCANPacketFile) != NULL)
	while(1)
	{
		if (gCurPktCount >= gNumTestFrames)
		{
			// Hit the number of packets we want to send. Stop.
			break;
		}

		// Get the next line.
		if (fgets(dataRead, 1024, pCANPacketFile) == NULL)
		{
			// Have not sent the requested number of packets.
			// Rewind the data file and start over.
			rewind(pCANPacketFile);

			if (fgets(dataRead, 1024, pCANPacketFile) == NULL)
			{
				// Empty file?
				break;
			}
		}

		if ((dataRead[0] == '#') || (dataRead[0] == '\n'))
		{
			// Found a commented line. Skip it.
			continue;
		}

		// Found a packet line. Parse and send it.
		if(gCmdVerboseFlag >= 1)
			printf("\n----->%s", dataRead);

		/* ***** Parse the packet line. ***** */
		curToken = strtok(dataRead, " ");
		while (curToken != (char *)NULL)
		{
			if (*curToken == '\n')
			{
				// Newline.
				break;
			}

			// Initialization code.
			curPacket.t0 = 0;
			curPacket.t1 = 0;
			extendedId	 = 0;  // Assume not extended ID.
			canFDEnabled = 0;
			extId		 = MAX_EXT_ID;

			for (int i = 0; i < TX_BUF_SIZE; i ++)
			{
				gOutBuf[i] = 0;
			}
			canLengthCode = 0;;

			// For this example:
			// 	All frames are CAN FD frames, bit-rate switching, ESI not set.
			// First 4 fields are set.
			curPacket.t0 = curPacket.t0 & 0x00000000;
			//curPacket.t1 |= CAN_RX_TX_CAN_FD_FORMAT;
			//curPacket.t1 |= CAN_RX_TX_BR_SWITCHING;

			/* Field 1. */
			// RTR (Remote transmition request) => t0, bit 29
			if (strncmp(curToken, "r", 1) == 0)
			{
				curPacket.t0 |= CAN_RX_TX_REMOTE_FRAME;
			}

			// Next token.
			curToken = strtok(NULL, " ");

			/* Field 2. */
			// XTD (Extended identifier) => t0, bit 30
			if (strncmp(curToken, "e", 1) == 0)
			{
				curPacket.t0 |= CAN_RX_TX_EXTENDED_ID;
				extendedId	 = 1;
			}
			else
			{
				// Assume if not extended, then standard.
				extendedId = 0;
			}

			// Next token.
			curToken = strtok(NULL, " ");
	
			/* Field 3 - NEW - CAN or CAN-FD. */
			// Values are s => Standard frame, f => CAN FD frame.
			if (strncmp(curToken, "f", 1) == 0)
			{
				canFDEnabled = 1;
				curPacket.t1 |= CAN_RX_TX_CAN_FD_FORMAT;
			}
			else
			{
				canFDEnabled = 0;
			}

			// Next token.
			curToken = strtok(NULL, " ");
	
			/* Field 4 - NEW - Valid only for CAN-FD, bit rate switching. */
			// Values are b => bit rate switching, n => no brs.
			if ((strncmp(curToken, "b", 1) == 0) && (canFDEnabled))
			{
				curPacket.t1 |= CAN_RX_TX_BR_SWITCHING;
			}

			// Next token.
			curToken = strtok(NULL, " ");
	
			// Now we get into variable length strings.
			// ID (Identifier) => t0
			// Extended identifier => 28:0
			// Standard identifier => 28:18
			tmpInt = (uint8_t)atoi(curToken);
			sscanf(curToken, "%x", &extId);
			if (extendedId == 1)
			{
				// Extended identifier.
				curPacket.t0 = CAN_RX_TX_SET_EXID(curPacket.t0, extId);
			}	
			else
			{
				// Standard identifier.
				curPacket.t0 = CAN_RX_TX_SET_ID(curPacket.t0, extId);
			}
	
			// Next token is data length.
			curToken = strtok(NULL, " ");

			tmpInt = atoi(curToken);
			curPacket.t1 &= ~CAN_RX_TX_SIZE_64;
			canLengthCode = tmpInt << 16;
    		curPacket.t1 |= canLengthCode;

			// Data - number of data bytes = length.
			int numData;
			numData = tmpInt;
			int curDataTok;
			for (curDataTok = 0; curDataTok < numData; curDataTok ++)
			{
				curToken = strtok(NULL, " ");
				sscanf(curToken, "%x", (unsigned int *)&tmpInt);
				gOutBuf[curDataTok] = tmpInt;
			}

			// Put data into the tx buffer.
			memcpy(curPacket.mcanTxData, gOutBuf, numData);

			// Done parsing this packet.
			// Send the packets.
			returnCode = VSL_CANTransmit(0, gTxPortNum, &curPacket);
			if (returnCode == VL_API_OK)
			{
				if(gCmdVerboseFlag >= 1)
				{
					printf("Successfully sent packet.\n");
				}

				gCurPktCount++;
			}
			else
			{
				printf("FAILED to send packet.\n");
				gFailedPktCount++;
			}

			// Get the next packet (line) in the file.
			break;

		}	// while (curToken ...)
	}  // fgets (dataRead ...)
	/* ********************************** */

	/* ***** Print some results. ***** */
	printf("---------------------------------------- \n");
	printf("SENDER_NUMBER_PKTS_SENT=%d\n", gCurPktCount);
	printf("RECEIVER_NUMBER_GOOD_PKTS_RECEIVED=%d\n", gCurRxPktCount); 
	printf("RECEIVER_NUMBER_ERR_PKTS_RECEIVED=%d\n", (gNumTestFrames - gCurPktCount));
	if(gCmdVerboseFlag >= 1)
	{
		printf("RECEIVER_TIMED_OUT_PKTS_RECEIVED=%d\n", gTimeOutPktCount); 
	}
	printf("---------------------------------------- \n");
	/* ******************************* */


	/* ***** Done. Clean up. ***** */
	if (gAPIDebugLevel >= VL_DEBUG_1)
		printf("Leaving sendAllPackets()\n");

	return;
	/* *************************** */
}


/* ***** Function to display application help message. ***** */
void showUsage()
{
	printf("\nVlCanApiTxRxExample version: %s\n", VER_STRING);
	printf( "Usage:   VlCanApiTxRxExample [rtadcvhfm]\n" );
    printf( "Options: -h   Displays usage info.\n" );
    printf( "         -v   Verbose output for selected command line options. Default: Very little output.\n" );
    printf( "         -d <n>  Data bit rate (<= 5000000). Default: 5000000.\n" );
    printf( "         -a <n>  Arbitration bit rate (<= 1000000). Default: 1000000.\n" );
    printf( "         -c <n>  Number of test frames. Default: 100.\n" );
    printf( "         -r <1|0>  Receive port. Default: 0.\n" );
    printf( "         -t <1|0>  Transmit port. Default: 1.\n" );
    printf( "         -f <CAN_DATA_FILE_NAME> text file containing CAN packet information.\n" );
    printf( "         -m 	Display the MCU firmware version information in xx.yy.zz format\n" );
    printf( "\n" );

	exit(-1);
}
/* ********************************************************* */

/* ***** Function to print error messages. ***** */
void printErrorCode(VL_APIStatusT errorCode)
{
	switch(errorCode)
	{
		case VL_API_OK:							printf("VL_API_OK\n"); break;
		case VL_API_ERROR:						printf("VL_API_ERROR\n"); break;
		case VL_API_INVALID_ARG:				printf("VL_API_INVALID_ARG\n");  break;
		case VL_API_NOT_SUPPORTED:				printf("VL_API_INVALID_ARG\n");  break;
		case VL_API_ARG_NOT_SUPPORTED:			printf("VL_API_ARG_NOT_SUPPORTED\n");  break;
		case VL_API_BC_LIBRARY_INIT_ERROR:		printf("VL_API_BC_LIBRARY_INIT_ERROR\n"); break;
		case VL_API_BC_LIBRARY_INSTALL_ERROR:	printf("VL_API_BC_LIBRARY_INSTALL_ERROR\n");  break;
		case VL_API_BC_BOARD_OPEN_ERROR:		printf("VL_API_BC_BOARD_OPEN_ERROR\n"); break;
		case VL_API_BC_LIBRARY_CLOSE_ERROR:		printf("VL_API_BC_LIBRARY_CLOSE_ERROR\n"); break;
		case VL_API_I2C_REQUESTED_BUS_NOT_FOUND:printf("VL_API_I2C_REQUEST_BUS_NOT_FOUND\n");  break;
		case VL_API_I2C_READ_ERROR:				printf("VL_API_I2C_READ_ERROR\n");  break;
		case VL_API_I2C_READ_REGISTER_ERROR:	printf("VL_API_I2C_READ_REGISTER_ERROR\n"); break;
		case VL_API_I2C_WRITE_ERROR:			printf("VL_API_I2C_WRITE_ERROR\n"); break;
		case VL_API_I2C_WRITE_REGISTER_ERROR:	printf("VL_API_I2C_WRITE_REGISTER_ERROR\n"); break;
    	case VL_API_GET_UPTIME_ERROR:			printf("VL_API_GET_UPTIME_ERROR\n"); break;
		case VL_API_VGA_BL_VALUE_OUT_OF_RANGE:	printf("VL_API_VGA_BL_VALUE_OUT_OF_RANGE\n"); break;
		case VL_API_VGA_BL_INTERFACE_ERROR:		printf("VL_API_VGA_BL_INTERFACE_ERROR\n"); break;
		case VL_API_IRQ_NUM_UNKNOWN:			printf("VL_API_IRQ_NUM_UNKNOWN\n"); break;
		case VL_API_CAN_CLOSE_DEVICE_ERROR:		printf("VL_API_CAN_CLOSE_DEVICE_ERROR\n"); break;
		case VL_API_CAN_OPEN_DEVICE_ERROR: 		printf("VL_API_CAN_OPEN_DEVICE_ERROR\n"); break;
		case VL_API_CAN_OPEN_NO_DEVICE_FOUND: 	printf("VL_API_CAN_OPEN_NO_DEVICE_FOUND\n"); break;
		case VL_API_CAN_OPEN_NO_DEVICE_DESC: 	printf("VL_API_CAN_OPEN_NO_DEVICE_DESC\n"); break;
		default:								printf("VL_UNKNOWN_ERROR_CODE\n");
	}
}

/*****************************************************************************/
/***  signalHandler - CTRL-C   ***********************************************/
/*****************************************************************************/
static void
signalHandler(
    int         signalNum
)
{
    gAbortFlag = true;

    return;
}

/* ********************************************* */

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
	vl_ReturnStatus = VSL_CANOpenPort(VL_CAN_BOARD0, gTxPortNum, gBitRateArb, 
									  gBitRateData, VL_ENABLE_RX_ON_TX, VL_DISABLE_LOOPBACK);

	if (vl_ReturnStatus != VL_API_OK)
	{
		printf("Retrieving MCU FW version: ERROR: ");
		printErrorCode(vl_ReturnStatus);
		printf("\n");
	}

	// Returning a value just in case we need it later.
	vl_ReturnStatus = VSL_CANGetMCUFwVersion(0, &fwVer);
	printf("%02x.%02x.%02x;\n", (fwVer >> 16), ((fwVer >> 8) & 0xF), (fwVer & 0xF));

	// Return value here does not really matter.
	vl_ReturnStatus = VSL_CANClosePort(VL_CAN_BOARD0, gTxPortNum);
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
    unsigned long vl_ReturnStatus;	// Return status.
    
    int			   opt;				// Command line option helper.

	uint32_t fwVer;
	struct sigaction	sigact;		// CTRL-C processing.

	unsigned char versaAPI_majorRev = (unsigned char)0;
	unsigned char versaAPI_minorRev = (unsigned char)0;
	unsigned char versaAPI_relRev   = (unsigned char)0;
    /* ********************************** */

    /* ***** Variable initializations. ***** */
    vl_ReturnStatus = 0xffffff;	// Assume a failure on open.
	opt				= 0;  // Process command line options.
    /* ************************************* */

    /* ***** Process command line options. ***** */
    while ((opt = getopt(argc, argv, "mf:c:v:r:t:a:d:h")) != -1)
    {
		switch(opt)
		{
			case 'f':	strcpy(gInputFileName, optarg);
						pCANPacketFileName = gInputFileName;
						break;
			case 'c':	gNumTestFrames = atoi(optarg);
						break;
			case 'v':  
						// Verbose mode.
						gCmdVerboseFlag = atoi(optarg);
						break;
			case 'r':
						// Rx port.
						gRxPortNum = atoi(optarg);
						break;
			case 't':
						// Tx port.
						gTxPortNum = atoi(optarg);
						break;
			case 'a':
						// Arbitration bit rate.
						gBitRateArb = atoi(optarg);
						break;
			case 'd':
						// Data bit rate.
						gBitRateData = atoi(optarg);
						break;
			case 'm':
						// Display the MCU version number in MM.mm.rr format.
						displayMCUVersionNumber();
						exit(0);

			case 'h':
			default:	
						showUsage();
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

	struct libusb_version *pLUVersion;
    pLUVersion = libusb_get_version();
	printf("\nVlCanApiTxRxExample version: %s\n", VER_STRING);
	printf("VlCanApiTxRxExample - Transmitting %d packets out CAN port %d;", gNumTestFrames, gTxPortNum);
	printf(" Receiving on CAN port %d;\n", gRxPortNum);
	printf("Bit Rates: Arbitration=%d; Data=%d;\n\n", gBitRateArb, gBitRateData);
	printf("CAN data file name: %s;\n", gInputFileName);

	VL_GetVersion(&versaAPI_majorRev, &versaAPI_minorRev, &versaAPI_relRev);
	printf("VersaAPI library version: %x.%x.%x\n", 
		versaAPI_majorRev, versaAPI_minorRev, versaAPI_relRev);
    printf("LIBUSB: %s%d.%d.%d.%d\n", 
		pLUVersion->rc, pLUVersion->major, pLUVersion->minor, 
		pLUVersion->micro, pLUVersion->nano );

	/* ***** Turn debug on. ***** */
    VSL_DebugLevelSet(gCmdVerboseFlag);
	/* ************************** */

	/* ***** Report some system information. ***** */
	if (gAPIDebugLevel >= VL_DEBUG_1)
	{
    	printf("\tLIBUSB: %s%d.%d.%d.%d\n", 
			pLUVersion->rc, pLUVersion->major, pLUVersion->minor, 
			pLUVersion->micro, pLUVersion->nano );
	}
	/* ******************************************* */

	/* ***** Make sure all options are set that need to be set. ***** */
	if ((gTxPortNum < 0) || (gTxPortNum > 1))
	{
		printf("Transmit port number must be 0 or 1;\n");
		showUsage();
		exit(-1);
	}

	if ((gRxPortNum < 0) || (gRxPortNum > 1))
	{
		printf("Receive port number must be 0 or 1;\n");
		showUsage();
		exit(-1);
	}

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

	/* ***** Open the ports per command line. ***** */
	// Transmit port.
	printf("Opening TX port %d ...", gTxPortNum);
	vl_ReturnStatus = VSL_CANOpenPort(VL_CAN_BOARD0, gTxPortNum, gBitRateArb, 
									  gBitRateData, VL_DISABLE_RX_ON_TX, VL_DISABLE_LOOPBACK);
									  //gBitRateData, VL_ENABLE_RX_ON_TX, VL_DISABLE_LOOPBACK);

	if (vl_ReturnStatus != VL_API_OK)
	{
		printf("... ERROR:");
		printErrorCode(vl_ReturnStatus);
		printf("\n");
	}
	else
	{
		printf("... OPENED;\n");
	}

	// Receive port.
	printf("Opening RX port %d ...", gRxPortNum);
	vl_ReturnStatus = VSL_CANOpenPort(VL_CAN_BOARD0, gRxPortNum, gBitRateArb, 
									  gBitRateData, VL_ENABLE_RX_ON_TX, VL_DISABLE_LOOPBACK);
									  //gBitRateData, VL_DISABLE_RX_ON_TX, VL_DISABLE_LOOPBACK);

	if (vl_ReturnStatus != VL_API_OK)
	{
		printf("... ERROR:");
		printErrorCode(vl_ReturnStatus);
		printf("\n");
	}
	else
	{
		printf("... OPENED;\n");
	}
	/* ******************************************** */

	/* ***** Print version number of the MCU code. ***** */
	fwVer = 0;

	// Returning a value just in case we need it later.
	vl_ReturnStatus = VSL_CANGetMCUFwVersion(0, &fwVer);
	printf("MPEu-C1E FW version=0x%06x;\n", fwVer);
	/* ************************************************* */

	/* ***** Send the desired number of packets. ***** */
	// Open the CAN data file. 
	pCANPacketFile = fopen(pCANPacketFileName, "r");
	if (pCANPacketFile == NULL)
	{
		// Error opening intput file containing packet information.
		if (gAPIDebugLevel >= VL_DEBUG_1)
			printf("Error opening input packet file %s\n", pCANPacketFileName);

		return(VL_API_INVALID_ARG);
	}
	else
	{
		// Successfully opend file containing packet information.
		if (gAPIDebugLevel >= VL_DEBUG_1)
			printf("Opened input packet file %s\n", pCANPacketFileName);
	}
	/* ******************************* */

	// Send the data.
	sendAllPackets();
	/* *********************************************** */

	/* ***** Done. ***** */
    return(0);
	/* ***************** */
}

