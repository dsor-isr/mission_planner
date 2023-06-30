/* VersaLogic VersaAPI Sample Code - 20160225
*
* This code is given with no guarantees and is intended to be used as an 
* example of how to use the VersaAPI software package and to check the 
* operation post install. This code will  work with various VersaLogic 
* single board computers and the VersaLogic VL-MPEe-A1/A2 miniPCIe board.
*
* Default Program Description: 
* This application will report the library version number and then toggle 
* an on-board DIO from HIGH to Low every three seconds for 30 seconds. 
* 
* Command Line Options: 
* Using the -a command line option a DIO on the MPEe-A1/A2 will be toggled 
* instead of the on-board DIO. 
* 
* This code has been compiled and run using gcc on Ubuntu 14.04.01, 
* kernel 3.19.0-25-generic, with VersaAPI version 1.3.1.  A Linux
* compatible Makefile has been provided with this .c file for an example
* of how to compile this code. */


/* ***** Includes Files. ***** */
// System.
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>

// VersaAPI header.
//#include "VL_CAN.h"
#include "VL_OSALib.h"
/* *************************** */

/* ***** Defines. ***** */
#define SLEEP_TIME 3 		// Time to sleep between setting the DIO.
#define LOOP_COUNT 3		// Number of times to toggle the DIO.
#define VL_RETURN_SUCCESS 0	// A successful return status from a VL API call.

#define PCR_REGISTER 0x00	// FPGA offset of the PCR register.
#define AUXMODE2 0x2B		// FPGA offset of the AUXMODE2 register.

#define YES 1				// Define yes.
#define NO  0				// Define no.
#define false (0)
#define true  (1)

#define SIG_VL_GPIO 56		// VersaLogic 8256 Signal identifier.
#define TX_BUF_SIZE         ( 64 )

#define MAX_STD_ID          (0x7FF)
#define MAX_EXT_ID          (0x1FFFFFFF)

#define BITRATE_ARBITRATION (1000000)
#define BITRATE_DATA        (5000000)

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

struct	sigaction	sigact;

int 		gCurRxPktCount	= 0;			// Current number of packets received.
int			gCurTxRxPktCount	= 0;		// Number of Tx'ed packets echo back.
int			gFailedPktCount	= 0;			// Assume all is ok.
int			gCANPort		= 0;			// Default CAN Port to receive on.
int			gCmdVerboseFlag		= 0;			// Verbosity off.
uint32_t    gBitRateArb			= BITRATE_ARBITRATION;
uint32_t    gBitRateData		= BITRATE_DATA;
bool		gAbortFlag			= false;	// Keep running.
/* ***************************** */

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

/*****************************************************************************/
/***  signalHandler - CTRL-C   ***********************************************/
/*****************************************************************************/
// bool        gAbortFlag = false;

static void signalHandler(int signalNum)
{

	VL_APIStatusT vl_ReturnStatus;
	vl_ReturnStatus = VSL_CANClosePort(VL_CAN_BOARD0, gCANPort);
	if (vl_ReturnStatus != VL_API_OK)
	{
		printf("ERROR: Could not closed CAN Device;\n");
	}
	else
	{
		printf("SUCCESS: Closed CAN Device;\n");
	}

	gAbortFlag = true;

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


static void displayMCUVersionNumber()
{
	/* ***** Declarations. ***** */
	//unsigned long vl_ReturnStatus;	// Return status.
	VL_APIStatusT vl_ReturnStatus;	// Return status.
	uint32_t fwVer;
	/* ************************* */

	/* ***** Initializations. ***** */
	vl_ReturnStatus	= VL_API_OK;
	fwVer			= (uint32_t)0;
	/* **************************** */

	/* ***** Retrieve the MCU firmware version. ***** */
	// Need to open a channel to the MCU.
	vl_ReturnStatus = VSL_CANOpenPort(VL_CAN_BOARD0, gCANPort, gBitRateArb, 
									  gBitRateData, VL_ENABLE_RX_ON_TX, VL_DISABLE_LOOPBACK);

	if (vl_ReturnStatus != VL_API_OK)
	{
		printf("Retrieving MCU FW version: ERROR: ");
		printErrorCode(vl_ReturnStatus);
		printf("\n");
	}

	// Returning a value just in case we need it later.
	vl_ReturnStatus = VSL_CANGetMCUFwVersion(0, &fwVer);
	printf("%02x.%02x.%02x;\n", (fwVer >> 16), (fwVer >> 8), (fwVer & 0xF));

	// Return value here does not really matter.
	vl_ReturnStatus = VSL_CANClosePort(VL_CAN_BOARD0, gCANPort);
	/* ********************************************** */

	/* ***** Done. ***** */
	return;
	/* ***************** */
}


/* ***** Function to display application help message. ***** */
void showUsage()
{
	printf( "\nUsage:   vlCanMfgTest [adhmpv]\n" );
    printf( "Options: -h   Displays usage info.\n" );
    printf( "         -v   Verbose output for selected command line options. Default: Very little output.\n" );
    printf( "         -d <n>  Data bit rate (<= 5000000). Default: 5000000.\n" );
    printf( "         -a <n>  Arbitration bit rate (<= 1000000). Default: 1000000.\n" );
    printf( "         -p <1|0>  Receive port. Default: 0.\n" );
    printf( "         -m 	Display the MCU firmware version information in xx.yy.zz format\n" );
    printf( "\n" );

	exit(-1);
}
/* ********************************************************* */

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


// User Defined function called when a packet is recieved.
//void userDefinedRxFunction(VL_USB_CAN_XFER *pRxPacket, int txStatus)
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
	}
	else
	{
		gFailedPktCount ++;
	}
	/* ******************** */

    if (gAPIDebugLevel >= VL_DEBUG_2)
		printf("Leaving app userDefinedRxFunction()\n");

	return;
}


/* ********************************************* */


/* ***** main program. ***** */
int main(int argc, char *argv[])
{
    /* ***** Variable declarations. ***** */
    //unsigned long vl_ReturnStatus;	// Return status.
    VL_APIStatusT vl_ReturnStatus;	// Return status.
    
    int			   opt;				// Command line option helper.

	unsigned char versaAPI_majorRev = (unsigned char)0;
	unsigned char versaAPI_minorRev = (unsigned char)0;
	unsigned char versaAPI_relRev   = (unsigned char)0;
    /* ********************************** */

    /* ***** Variable initializations. ***** */
    vl_ReturnStatus = 0xffffff;	// Assume a failure on open.
	opt				= 0;  // Process command line options.
    /* ************************************* */

    /* ***** Process command line options. ***** */
	while ((opt = getopt(argc, argv, "mf:v:p:a:d:h")) != -1)
    {
		switch(opt)
		{
			case 'v':  
						// Verbose mode.
						gCmdVerboseFlag = atoi(optarg);
						break;
			case 'a':
						// Arbitration bit rate.
						gBitRateArb = atoi(optarg);
						break;
			case 'd':
						// Data bit rate.
						gBitRateData = atoi(optarg);
						break;
			case 'p':
						// CAN port to receive on.
						gCANPort = atoi(optarg);
						break;
			case 'm':
						// Display the MCU version number in MM.mm.rr format.
						displayMCUVersionNumber();
						exit(1);

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

	/* ***** Print example information. ***** */
	const struct libusb_version *pLUVersion;
    pLUVersion = libusb_get_version();
	printf("\nVersaLogic MPEu-C1E Packet Receive Example Application\n");

	printf("Port Configuration (MBits/s):\n");
	printf("\tPort number:\t\t%d\n\tArbitration bit rate:\t%d\n\tData bit rate:\t\t%d\n\n", 
		gCANPort, gBitRateArb, gBitRateData);

	// VersaAPI version.
	printf("System Information:\n");
	VL_GetVersion(&versaAPI_majorRev, &versaAPI_minorRev, &versaAPI_relRev);
	printf("\tVersaAPI Version: %x.%x.%x\n", 
		versaAPI_majorRev, versaAPI_minorRev, versaAPI_relRev);

	// libusb version.
    printf("\tLIBUSB Version: %s%d.%d.%d.%d\n\n", 
		pLUVersion->rc, pLUVersion->major, pLUVersion->minor, 
		pLUVersion->micro, pLUVersion->nano );
	/* ************************************** */


	/* ***** Turn debug on/off. ***** */
    VSL_DebugLevelSet(gCmdVerboseFlag);
	/* ****************************** */


	/* ***** Make sure all options are set that need to be set. ***** */
	if ((gBitRateArb < 0) || (gBitRateArb > 1000000))
	{
		printf("Invocation Error: Arbitration bit rate must be > 0 and < 1000000;\n");
		showUsage();
		exit(-1);
	}

	if ((gBitRateData < 0) || (gBitRateData > 5000000))
	{
		printf("Invocation Error: Data bit rate must be > 0 and < 5000000;\n");
		showUsage();
		exit(-1);
	}

	if ((gCANPort < VL_CAN_PORT0) || (gCANPort > VL_CAN_PORT1))
	{
		printf("Invocation Error: CAN Port to receive must be %d, or %d\n", VL_CAN_PORT0, VL_CAN_PORT1);
		showUsage();
		exit(-1);
	}
	/* ************************************************************** */


	/* ***** Open a CAN device. ***** */
	vl_ReturnStatus = VSL_CANOpenPort(VL_CAN_BOARD0, gCANPort, gBitRateArb, 
		  							  gBitRateData, VL_ENABLE_RX_ON_TX, 
									  VL_DISABLE_LOOPBACK);
	if (vl_ReturnStatus != VL_API_OK)
	{
		printf("Error: Cound not open CAN Port %d:", gCANPort);
		printErrorCode(vl_ReturnStatus);
		exit(1);
	}
	else
	{
		printf("SUCCESS: Opened CAN Port %d for receiving\n", gCANPort);
	}
	/* ****************************** */


	/* ***** Loop forever reporting on rx packets. ***** */
	int rtn = 0;
	//while( gAbortFlag == false )
	while( 1 )
    {
        rtn = libusb_handle_events( NULL );
        if ( LIBUSB_SUCCESS != rtn )
        {
            printf( "\nERROR: libusb_handle_events %s\n", libusb_error_name( rtn ) );
            break;
        }
    }
	/* ************************************************* */

	/* ***** Close a CAN device. ***** */
	printf("Attempting to close CAN Port %d;\n",
	gCANPort);

	vl_ReturnStatus = VSL_CANClosePort(VL_CAN_BOARD0, gCANPort);
	if (vl_ReturnStatus != VL_API_OK)
	{
		printf("ERROR: Could not closed CAN Device;\n");
	}
	else
	{
		printf("SUCCESS: Closed CAN Device;\n");
	}

	/* ***** Done. ***** */
    return(0);
	/* ***************** */
}

