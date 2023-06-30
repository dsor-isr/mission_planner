/* VersaLogic rx_tx sample application
*
* This code is given with no guarantees and is intended to be used as an 
* example of how to use the VersaAPI software package and to check the 
* operation post install.
*
* Default Program Description: 
* -----=====++++++ START NEEDS UPDATED +++++=====------
* Requires that the two CAN ports be connected to each other. One CAN port
* will be the receive port (-r) the other CAN port will be the transmit (-t) port.
* Packets will be read out of a text file (see example packet file), trasmitted
* out the tx port and received on the rx port. A count of the number of packets
* tranmitted, the number of packets received and the number of packets that
* timeout is reported at the end of the test. The number of packets transmitted
* is set by the -c command line option.
* -----=====++++++ END NEEDS UPDATED +++++=====------
* 
* Command Line Options: 
* -----=====++++++ START NEEDS UPDATED +++++=====------
* -----=====++++++ END NEEDS UPDATED +++++=====------
* 
* This code has been compiled and run using gcc on Ubuntu 18.04.04 LTS, 
* kernel x.y.z-aa-generic, with VersaAPI CAN library 1.1.e.  A Linux
* compatible Makefile has been provided with this .c file for an example
* of how to compile this code. 

Revision	1.0	20201001:	-Initial version.
Revision	1.1	20210125:	-Added calls to new API functions: 
                                 VSL_CANOpenPortWithUserTiming()
								 VSL_CANGetProtocolRegisterStatus()
								 VSL_CANGetErrorCounterRegisterStatus()
							-Added command line options to support
							 new API calls.
							-Rearranged order of some calls to simplify
							 the code.
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
#define VER_STRING "1.1"	// Version string of this application.

#define false (0)
#define true  (1)

#define TX_BUF_SIZE         ( 64 )
#define MAX_EXT_ID          (0x1FFFFFFF)
#define UNKNOWN_PORT_NUM    (2)

#define BITRATE_ARBITRATION (1000000)
#define BITRATE_DATA        (2000000)
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
char	*pCANPacketFileName	= "./CANPacketsToTx.txt";
FILE	*pCANPacketFile		= (FILE *)NULL;
MCAN_TX_PACKET	curPacket;

uint8_t		gOutBuf[TX_BUF_SIZE];
char		gInputFileName[128];

int 		gCurPktCount	= 0;			// Current number of packets sent.
int 		gCurRxPktCount	= 0;			// Current number of packets received.
int			gCurTxRxPktCount= 0;			// Number of Tx'ed packets echo back.
											// Controlled by OpenPort API call.
int			gNumTestFrames	= NUM_FRAMES;	// Number of packets to send.
int			gFailedPktCount	= 0;			// Assume all is ok.
int			gCmdVerboseFlag	= 0;			// Verbosity off.
bool        gAbortFlag 		= false;
uint32_t    gBitRateArb		= BITRATE_ARBITRATION;
uint32_t    gBitRateData	= BITRATE_DATA;
int         gTxPortNum		= UNKNOWN_PORT_NUM;	// 0, 1 - Valid ports. Default unknown.
int         gRxPortNum		= UNKNOWN_PORT_NUM;	// 0, 1 - Valid ports. Default unknown.
int         gPrintPSRInfo   = false;            // Print the contents of the PSR register 
                                                // after every tx.
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


// Function to dump upto the first 16 data bytes of a packet.
void dumpData(uint8_t* pData, int lengthOfData)
{
	int dataIndex = 0;

    printf("\tData Dump:\t");
	while ((dataIndex < lengthOfData) && (dataIndex < 16))
	{
        printf("%02X ", pData[dataIndex]);
		dataIndex ++;
	}
    printf("\n");

}


/* ****************************************************************************
//	CANPrintPacket()
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
//
// Need to keep updated with the number of available commands.
   ************************************************************************** */
void CANPrintPacket(VL_USB_CAN_XFER *pPacket)
{
    int	frameSize;
	int packetFD = 0; // Assume standard CAN packet.

    if ( pPacket->cmd >= 0 && pPacket->cmd <= USB_CAN_CMD_MAX )
    {
        printf( "Command:%s; Size:%d; Port:%d\n",
		  pCmdStr[pPacket->cmd],
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
			// Nothing of interest to print for these.
        	printf( "\n" );
        	break;

    	case USB_CAN_CMD_OPEN:
        	printf("\tarb rate: %ld\n", (long)pPacket->u.openInfo.arbitrationRate);
        	printf("\tdata rate: %ld\n", (long) pPacket->u.openInfo.dataRate);
        	printf("\tloop back: %d\n\n", pPacket->u.openInfo.loopback ? 1 : 0);
        	break;

    	case USB_CAN_CMD_TX:
    	case USB_CAN_CMD_RX:
			// Is it a CAN FD packet?
			packetFD = pPacket->u.txPacket.t1 & CAN_RX_TX_CAN_FD_FORMAT;

        	// T0 and R0
        	if ( pPacket->u.txPacket.t0 & CAN_RX_TX_EXTENDED_ID )
        	{
            	printf("\tExt Id:\t\t0x%08X\n", CAN_RX_TX_GET_EXID( pPacket->u.txPacket.t0));
        	} else 
			{
            	printf("\tId:\t\t0x%03X\n", CAN_RX_TX_GET_ID( pPacket->u.txPacket.t0));
        	}
        	printf("\tSIF:\t\t%s\n", pPacket->u.txPacket.t0 & CAN_RX_TX_REMOTE_FRAME ? "Remote" : "Data");
        	printf("\tESI:\t\t%s\n", pPacket->u.txPacket.t0 & CAN_RX_TX_ERR_PASSIVE ? "Passive" : "Active" );

        	// T1 and R1
        	frameSize = mcan_length_to_size( (pPacket->u.txPacket.t1 >> 16) & 0xF );
        	printf("\tData Length:\t%d\n", frameSize );
			if (packetFD)
			{
        		printf("\tBRS:\t\t%s\n", pPacket->u.txPacket.t1 & CAN_RX_TX_BR_SWITCHING ? "On" : "Off" );
			}
        	printf("\tFormat:\t\t%s\n", packetFD ? "CAN FD" : "CAN" );

        	if ( USB_CAN_CMD_RX == pPacket->cmd )
        	{
            	printf("\tTime Stamp:\t%d\n", pPacket->u.rxPacket.r1 & 0xFFFF );
            	printf("\tFilter Index:\t%d\n", (pPacket->u.rxPacket.r1 >> 26) & 0x7F );
            	printf("\tFrame Match:\t%d\n", (pPacket->u.rxPacket.r1 & 0x80000000) ? 1 : 0 );
        	} else {
            	printf("\tStore Events:\t%s\n", (pPacket->u.txPacket.t1 & 0x00800000) ? "yes" : "no" );
            	printf("\tMarker:\t0x%02X\n", (pPacket->u.txPacket.t1 & 0xFF000000) >> 24 );
        	}

			// For Fortem checking, if packet comes from 0x097 then send a response packet back.
			uint32_t tmpId = 0;
        	if (USB_CAN_CMD_RX == pPacket->cmd)
        	{
				// Get the packet ID
        	    if ( pPacket->u.txPacket.t0 & CAN_RX_TX_EXTENDED_ID )
        	    {
            	    tmpId = CAN_RX_TX_GET_EXID(pPacket->u.txPacket.t0);
        	    } 
			    else 
			    {
            	    tmpId = CAN_RX_TX_GET_ID(pPacket->u.txPacket.t0);
        	    }

				// Is it a packet of interest?
				if (0x97 == tmpId)
				{
					// Yes it is.
					// Process it specially.
					printf("\t\t***** FOUND SPECIAL PACKET (0x%08x) PROCESSING ... \n", tmpId);
				}
			}

        	// If there is data do a dump of it.
        	if ( frameSize )
        	{
            	// NOTE: mcanTxData and mcanRxData will be the same.
            	dumpData((uint8_t*) &(pPacket->u.rxPacket.mcanRxData), frameSize);
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

void printPSRRegStrings(uint32_t lec, uint32_t act , uint32_t ep, uint32_t ew, uint32_t bo,
						uint32_t dlec, uint32_t resi, uint32_t rbrs, uint32_t rfdf, 
						uint32_t pxe, uint32_t tdcv)
{
    printf("PSR Register:\n");
    printf("\tLast Error Code                                         (LEC=0x%03x)", lec);
	switch (lec)
	{
		case 0x0: 
		    printf(" No error since LEC bits reset by successful rx or tx\n");
			break;
		case 0x1: 
		    printf(" Stuff error. More than 5 equal bits in a seqence as part of a rx message\n");
			break;
		case 0x2: 
		    printf(" Form error. A fixed format part of a rx message has incorrect format\n");
			break;
		case 0x3: 
		    printf(" Ack error. Message tx was not acknowledged by another node\n");
			break;
		case 0x4: 
		    printf(" Bit1 error - tx message error\n");
			break;
		case 0x5: 
		    printf(" Bit0 error - tx message error\n");
			break;
		case 0x6: 
		    printf(" CRC error - CRC check sum of a rx message was incorrect\n");
			break;
		case 0x7: 
		    printf(" No change - Any read of the PSR resets LEC bits to 0x7\n");
			break;
		default:
		    printf(" INVALID PSR REGISTER READ VALUE!\n");
			break;
	}

    printf("\tActivity                                                (ACT=0x%02x) Node is ", act);
	switch (act)
	{
	    case 0x0: 
		    printf(" Synchronizing\n");
		    break;
	    case 0x1: 
		    printf(" Idle - neither rx'ing or tx'ing\n");
		    break;
	    case 0x2: 
		    printf(" Receiving\n");
		    break;
	    case 0x3: 
		    printf(" Transmitting\n");
		    break;
		default:
		    printf(" INVALID PSR REGISTER READ VALUE!\n");
			break;
	}

    printf("\tError passive                                           (EP=0x%1x) Node is in Error ", ep);
	if (ep == 0)
	{
		printf("ACTIVE state\n");
	}
	else
	{
		printf("PASSIVE state\n");
	}

    printf("\tWarning status                                          (EW=0x%1x) ", ew);
	if (ew == 0)
	{
		printf("Both error counters < Error_Warning limit of 96\n");
	}
	else
	{
		printf("At least one error counter > Error_Warning 0f 96\n");
	}

    printf("\tBus off status                                          (BO=0x%1x) Bus is ", bo);
	if (bo == 0)
	{
		printf("On\n");
	}
	else
	{
		printf("Off\n");
	}

    printf("\tData Phase Last Error Code                              (DLEC=0x%03x)", dlec);
	switch (lec)
	{
		case 0x0: 
		    printf(" No error since LEC bits reset by successful rx or tx\n");
			break;
		case 0x1: 
		    printf(" Stuff error. More than 5 equal bits in a seqence as part of a rx message\n");
			break;
		case 0x2: 
		    printf(" Form error. A fixed format part of a rx message has incorrect format\n");
			break;
		case 0x3: 
		    printf(" Ack error. Message tx was not acknowledged by another node\n");
			break;
		case 0x4: 
		    printf(" Bit1 error - tx message error\n");
			break;
		case 0x5: 
		    printf(" Bit0 error - tx message error\n");
			break;
		case 0x6: 
		    printf(" CRC error - CRC check sum of a rx message was incorrect\n");
			break;
		case 0x7: 
		    printf(" No change - Any read of the PSR resets LEC bits to 0x7\n");
			break;
		default:
		    printf(" INVALID PSR REGISTER READ VALUE!\n");
			break;
	}

    printf("\tESI flag of last received CAN FD message                (RESI=0x%1x) RESI ", resi);
	if (resi == 0)
	{
		printf("flag NOT set\n");
	}
	else
	{
		printf("flag IS set\n");
	}

    printf("\tBRS flag of last received CAN FD message                (RBRS=0x%1x) BRS flag ", rbrs);
	if (rbrs == 0)
	{
		printf("NOT set\n");
	}
	else
	{
		printf("IS set\n");
	}

    printf("\tReceived a CAN FD message since last CPU reset          (RFDF=0x%1x) ...", rfdf);
	if (rfdf == 0)
	{
		printf("...NO\n");
	}
	else
	{
		printf("...YES\n");
	}

    printf("\tProtocol exception event occured since last read access (PXE=0x%1x) ...", pxe);
	if (rfdf == 0)
	{
		printf("...NO\n");
	}
	else
	{
		printf("...YES\n");
	}

    printf("\tTransmit delay compensator value                        (TDCV=0x%07x)\n", tdcv);

	// Done.
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
	if (gAPIDebugLevel >= VL_DEBUG_2)
	{
		printf("Entering app userDefinedRxFunction(), txStatus=%d\n", txStatus);
	}	

	if(txStatus == LIBUSB_TRANSFER_COMPLETED)
	{
		if(pRxPacket->cmd == USB_CAN_CMD_RX)
		{
			gCurRxPktCount ++;
			gCurTxRxPktCount ++;
		
			(void)VSL_CANPrintPktHeader("Packet Received:", pRxPacket);
			CANPrintPacket(pRxPacket);
		}
		else if(pRxPacket->cmd == USB_CAN_CMD_MCU_REG_VAL)
		{
	        if (gAPIDebugLevel >= VL_DEBUG_2)
			{
			    printf("\tapp user_CANRxCallBackFunction(): Register 0x%d:value=0x%08x;\n", 
					pRxPacket->misc, pRxPacket->extendedStatus);
			}
		}
	}
	else
	{
		gFailedPktCount ++;
	}
	/* ******************** */

    if (gAPIDebugLevel >= VL_DEBUG_2)
		printf("Leaving app userDefinedRxFunction()\n");

	// Done.
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
	int             i;
	uint32_t		canLengthCode;

	uint32_t lec; 
	uint32_t act;
	uint32_t ep; 
	uint32_t ew; 
	uint32_t bo; 
	uint32_t dlec; 
	uint32_t resi; 
	uint32_t rbrs; 
	uint32_t rfdf; 
	uint32_t pxe; 
	uint32_t tdcv;
	
	uint32_t tec;
	uint32_t rec;
	uint32_t rp;
	uint32_t cel;

	int numData;
	/* ************************* */

	/* ***** Where are we. ***** */
	if (gAPIDebugLevel >= VL_DEBUG_1)
		printf("Entering sendAllPackets()\n");
	/* ************************* */

	/* ***** Initializations. ***** */
	gFailedPktCount	= 0;
	gCurPktCount	= 0;
	numData         = 0;
	/* **************************** */

	/* ***** Process each packet line in the input file. ***** */
	// Not at the end of the file. Get the next packet.
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
		printf("Packet #%d:--->%s", (gCurPktCount+1), dataRead);

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

			for (i = 0; i < TX_BUF_SIZE; i ++)
			{
				gOutBuf[i] = 0;
			}
			canLengthCode = 0;;

			// Clear the .t0 and .t1 fields.
			// Clear the transmit packet data field.
			for (i = 0; i < CAN_MAX_FRAME_SIZE; i ++)
			{
				curPacket.mcanTxData[i] = 0;
			}

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
			numData = tmpInt;

			// Use the correct DLC for the requested size (round up).
			// If tmpInt is <= 8, no need to change it.
			if ((tmpInt > 8) && (tmpInt <= 12))
			{
				tmpInt = 0x09;
			}
			else if ((tmpInt > 12) && (tmpInt <= 16))
			{
				tmpInt = 0x0a;
			}
			else if ((tmpInt > 16) && (tmpInt <= 20))
			{
				tmpInt = 0x0b;
			}
			else if ((tmpInt > 20) && (tmpInt <= 24))
			{
				tmpInt = 0x0c;
			}
			else if ((tmpInt > 24) && (tmpInt <= 32))
			{
				tmpInt = 0x0d;
			}
			else if ((tmpInt > 32) && (tmpInt <= 48))
			{
				tmpInt = 0x0e;
			}
			else if ((tmpInt > 48) && (tmpInt <= 64))
			{
				tmpInt = 0x0f;
			}

			// Stuff the t1 field.
			curPacket.t1 &= ~CAN_RX_TX_SIZE_64;
			canLengthCode = tmpInt << 16;
    		curPacket.t1 |= canLengthCode;

			// Data - number of data bytes = length.
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

			if(gPrintPSRInfo == true)
			{
			    // Get some MCU register values. 
			    VSL_CANGetProtocolRegisterStatus(0, gTxPortNum, &lec, &act,
				          &ep, &ew, &bo, &dlec, &resi, &rbrs, &rfdf, &pxe, &tdcv);
				printPSRRegStrings(lec, act, ep, ew, bo, dlec, resi, rbrs, rfdf, 
								   pxe, tdcv);
			}

			// Get the next packet (line) in the file.
			break;

		}	// while (curToken ...)
	}  // fgets (dataRead ...)
	/* ********************************** */

	/* ***** Print some results. ***** */
	printf("---------------------------------------- \n");
	printf("Number of CAN packets sent=%d\n", gCurPktCount);
	printf("Packet Error Counts:\n");
	(void)VSL_CANGetErrorCounterRegisterStatus(0, gTxPortNum, &tec, &rec,
	      &rp, &cel);
	printf("ECR Register:\n");
	printf("\tTransmit Error Counter (TEC)=%d\n", tec);
	printf("\tReceive Error Counter  (REC)=%d\n", rec);
	printf("\tCAN Error Logging      (CEL)=%d\n", cel);
	if (rp <= 0)
	{
	    printf("\tReceive Error Passive  (RP) =0x%1x is BELOW error level of 128\n", rp);
	}
	else
	{
	    printf("\tReceive Error Passive  (RP) =0x%1x is ABOVE error level of 128\n", rp);
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
	printf("\nrx_tx version: %s\n", VER_STRING);
	printf("  Send/Receive packets out/in the specified port. Whether the port is sending or receving\n");
	printf("  is determined by the -r and -t options. Each invocation of the sample application can either\n");
	printf("  send or receive, one invocation cannot do both.\n\n");
	printf( "Usage:   rx_tx [rtadcvhfm]\n" );
    printf( "Options: -h   Displays usage info.\n" );
    printf( "         -v <n>  Verbose output for selected command line options. Default: Very little output.\n" );
    printf( "         -d <n>  Data bit rate (<= 5000000). Default: 2000000.\n" );
    printf( "         -a <n>  Arbitration bit rate (<= 1000000). Default: 1000000.\n" );
    printf( "         -c <n>  Number of frames to send. Default: 100.\n" );
    printf( "         -r <1|0>  Receive port. Default: 0.\n" );
    printf( "         -t <1|0>  Transmit port. Default: 1.\n" );
    printf( "         -f <CAN_DATA_FILE_NAME> text file containing CAN packet information.\n" );
    printf( "         -m 	Display the MCU firmware version information in xx.yy.zz format\n" );
    printf( "         -p 	Print the contents of the PSR register after every transmitted packet.\n" );
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
static void signalHandler( int signalNum)
{
	// VL_APIStatusT vl_ReturnStatus;

	// Closing of the CAN Rx port done in main().

	// Set the Rx termination flag.
    gAbortFlag = true;

    return;
}

/* ********************************************* */


/* ***** Retreive the MCU Firmware version number. ***** 
   *
   * Query the MCU via port 0 and get the MCU firmware version.
   * Assumes the CAN port is already opened.
   *                                                              */
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
	// Assumption: The CAN Port has already been opened.

	// Returning a value just in case we need it later.
	vl_ReturnStatus = VSL_CANGetMCUFwVersion(0, &fwVer);
	if (vl_ReturnStatus != VL_API_OK)
	{
		printf("Retrieving MCU FW version: ERROR: ");
		printErrorCode(vl_ReturnStatus);
		printf("\n");
	}
	else
	{
	printf("MPEu-C1E FW version: %02x.%02x.%02x;\n\n", (fwVer >> 16), ((fwVer >> 8) & 0xF), (fwVer & 0xF));
	}
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

	struct sigaction	sigact;		// CTRL-C processing.

	unsigned char versaAPI_majorRev = (unsigned char)0;
	unsigned char versaAPI_minorRev = (unsigned char)0;
	unsigned char versaAPI_relRev   = (unsigned char)0;
	int           txPortSpecified   = false;
	int           rxPortSpecified   = false;
	int           getMCUVer         = false;
	int           curPort           = VL_CAN_PORTUNSPECIFIED;
    /* ********************************** */

    /* ***** Variable initializations. ***** */
    vl_ReturnStatus = 0xffffff;	// Assume a failure on open.
	opt				= 0;  // Process command line options.
    /* ************************************* */

    /* ***** Process command line options. ***** */
    while ((opt = getopt(argc, argv, "mf:c:v:r:t:a:d:hp")) != -1)
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
						curPort    = gRxPortNum;
						rxPortSpecified = true;
						break;
			case 't':
						// Tx port.
						gTxPortNum = atoi(optarg);
						curPort    = gTxPortNum;
						txPortSpecified = true;
						break;
			case 'p':
						// Tx port.
						gPrintPSRInfo = true;
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
						getMCUVer = true;
						break;

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
    sigaction(SIGINT, &sigact, NULL );
    sigaction(SIGTERM, &sigact, NULL );
    sigaction(SIGQUIT, &sigact, NULL );
	/* ************************************ */

	const struct libusb_version *pLUVersion;
    pLUVersion = libusb_get_version();
	printf("\nrx_tx version: %s\n", VER_STRING);
	printf("Bit Rates: Arbitration=%d; Data=%d;\n\n", gBitRateArb, gBitRateData);
	printf("CAN data file name: %s;\n", gInputFileName);

	VSL_CANGetVersion(&versaAPI_majorRev, &versaAPI_minorRev, &versaAPI_relRev);
	printf("VersaAPI library version: %x.%x.%x\n", 
		versaAPI_majorRev, versaAPI_minorRev, versaAPI_relRev);
    printf("LIBUSB: %s%d.%d.%d.%d\n\n", 
		pLUVersion->rc, pLUVersion->major, pLUVersion->minor, 
		pLUVersion->micro, pLUVersion->nano );

	/* ***** Turn debug on. ***** */
    VSL_DebugLevelSet(gCmdVerboseFlag);
	/* ************************** */


	/* ***** Make sure all options are set that need to be set. ***** */
	// Validate that there is a port set for either Tx or Rx.
	if ((txPortSpecified == false) && (rxPortSpecified == false) && (getMCUVer == false))
	{
	    printf("COMMAND LINE ERROR: Must specify either a Tx or Rx port number;\n");
	    showUsage();
	    exit(-1);
	}

	// At this point we know we have at least one port specified.
	if (txPortSpecified == true)
	{
	    if ((gTxPortNum < 0) || (gTxPortNum > 1))
		{
		    printf("COMMAND LINE ERROR: Transmit port number must be 0 or 1;\n");
		    showUsage();
		    exit(-1);
		}
	    else
	    {
	        printf("rx_tx - Transmitting %d packets out CAN port %d;\n", gNumTestFrames, gTxPortNum);
	    }
	}

	if (rxPortSpecified == true)
	{
	    if ((gRxPortNum < 0) || (gRxPortNum > 1))
	    {
		    printf("COMMAND LINE ERROR: Receive port number must be 0 or 1;\n");
		    showUsage();
		    exit(-1);
	    }
	    else
	    {
	        printf("Setting up to receive packets on CAN port %d;\n", gRxPortNum);
	    }
	}

	// Validate arbitration rate.
	if ((gBitRateArb < 0) || (gBitRateArb > 1000000))
	{
		printf("COMMAND LINE ERROR: Arbitration bit rate must be > 0 and < 1000000;\n");
		showUsage();
		exit(-1);
	}

	// Validate bit rate.
	if ((gBitRateData < 0) || (gBitRateData > 5000000))
	{
		printf("COMMAND LINE ERROR: Data bit rate must be > 0 and < 5000000;\n");
		showUsage();
		exit(-1);
	}
	/* ************************************************************** */


	/* ***** Open up the CAN port. Needed for either Rx or Rx. ***** */
	printf("Opening Port %d ...", curPort);
	/*
    vl_ReturnStatus = VSL_CANOpenPort(VL_CAN_BOARD0, gTxPortNum, gBitRateArb, 
								      gBitRateData, VL_DISABLE_RX_ON_TX, VL_DISABLE_LOOPBACK);
	*/
	vl_ReturnStatus = VSL_CANOpenPortWithUserTiming(VL_CAN_BOARD0, curPort,         
					              (uint32_t)1000000,            // Arbitration rate
								  (uint32_t)2000000,            // Data rate
								  (uint32_t)VL_ENABLE_RX_ON_TX, 
								  (uint32_t)VL_DISABLE_LOOPBACK,
								  (uint32_t)5,                                      // Master divider
								  (uint16_t)1, (uint8_t)3, (uint8_t)11, (uint8_t)4,  // arbitration timing.
								  (uint16_t)0, (uint8_t)3, (uint8_t)11, (uint8_t)4   // data timing.
								  );

	if (vl_ReturnStatus != VL_API_OK)
	{
	    printf("... ERROR:");
	    printErrorCode(vl_ReturnStatus);
	    printf("\n");
		return(-1);
	}
	else
	{
	    printf("... OPENED;\n");
	}
	/* ************************************************************* */


	/* ***** Print version number of the MCU code. ***** */
	displayMCUVersionNumber();
	/* ************************************************* */


	/* ***** Start sending and/or receiving packets. ***** */
	// If Tx and Rx are both specified, Tx will happen first sending the 
	// specified number of packets and then Rx will start, waiting for 
	// packets to come-in.
	// At this point we know the ports are valid port numbers.
	  
	// Start the transmit process if appropriate.
	if(txPortSpecified == true)
	{
	    printf("Starting TX Process on port %d:\n", gTxPortNum);
		  
	    // Transmitting packets.
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

	    // Send the data.
	    sendAllPackets();

	}
	else
	{
		printf("Tx not requested.\n");
	}

	// Start the receive process if appropriate.
	if(rxPortSpecified == true)
	{
		// Loop forever reporting on packets received.
		int rtn = 0;
		
		printf("\nWaiting to receive CAN packets on port %d ...\n", gRxPortNum);
	    while( gAbortFlag == false )
        {
            rtn = libusb_handle_events( NULL );
            if ( LIBUSB_SUCCESS != rtn )
            {
                printf( "\nERROR: libusb_handle_events %s\n", libusb_error_name( rtn ) );
                break;
            }

			// Allows for the signal to be trapped "nicely".
		    usleep(100000);
        }

		// Done receiving packets.
		// Report some statistics.
	    printf("---------------------------------------- \n");
	    printf("Number of good CAN packets received=%d\n", gCurRxPktCount); 
	    printf("---------------------------------------- \n");
	}
	else
	{
		printf("Rx not requested.\n");
	}
	/* *************************************************** */


    /* ***** Done. ***** */
	// Close the port.
	printf("Closing port %d ...", curPort);

	vl_ReturnStatus = VSL_CANClosePort(VL_CAN_BOARD0, curPort);
	if (vl_ReturnStatus != VL_API_OK)
	{
	    printf("ERROR: Could not closed CAN Port;\n");
	}
	else
	{
	    printf("... CLOSED;\n");
	}

    return(0);
    /* ***************** */
}

