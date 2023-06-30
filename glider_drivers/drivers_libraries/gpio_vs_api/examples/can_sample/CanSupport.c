/* MPEu-C1E helper functions.
*
* This code is given with no guarantees and is intended to be used as an 
* example of how to use the Linux native software packages to access
* the VersaLogic MPEu-C1E mPCIe CAN card.
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



#ifdef WIN32
#include <windows.h>
#include <winioctl.h>
#include <intrin.h>
#include "gpioctl.h"
#endif // WIN32

#include  <string.h>

#ifdef linux
#include "VL_CAN.h"
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>
#endif // linux

#define RX_BUF_SIZE         ( 512 )
#define TX_BUF_SIZE         ( 64 )

/* **** Global Variables. ***** */
struct deviceAccessInfo		*pDevInfo;
VL_USB_CAN_XFER             canRequest;
libusb_device				*pDevicesFound[MAX_NUM_USB_DEVICES]; // Array of VL USB devices.

bool 		gRxCancelled 	= false;
bool 		gTxCancelled = false;
bool 		gRxTimeOutExit	= false;
uint32_t	gRxTimeOutCount	= 0;
uint32_t	gRxPacketCount	= 0;

struct libusb_transfer	*pXferBulkIn	= NULL;
struct libusb_transfer	*pXferBulkOut	= NULL;

uint8_t	*pInBuf				= NULL;
// MM bool	gAbortFlag			= false;
bool	gUSBDeviceOpened 	= false;	// true => USB device is open to use, else false.
bool	gPort0Opened		= false;	// true => Port 0 is open and ready to use, else false.
bool	gPort1Opened		= false;	// true => Port 1 is open and ready to use, else false.
VL_USB_CAN_XFER	canCmd;  // Command sent to the CAN device via USB.
VL_USB_CAN_XFER	canSts;  // Command sent to the CAN device via USB.
VL_USB_CAN_XFER	*pTxTest;  // Pointer to a CAN packet.

uint8_t gOutBuf[TX_BUF_SIZE];

// Added 5/23/2019  MM
uint8_t			bulkOut;
uint8_t			bulkIn;

// Added 5/30/2019 MM
uint32_t	gMCUFwReceived;
uint8_t		gCANPortStatus;  // 0 => down; 1 => up;

bool	sendPacket		= true;
bool	rcvPacket		= true;
bool	txBusy			= false;
int		busErrorCount	= 0;

// Added 10/09/2019 MM
libusb_device_handle 	*gpDevHandle = (libusb_device_handle *)NULL;

/* **************************** */


/* ***** Defines. ***** */
//#define USB_CAN_CMD_MAX		(7)	// Number of commands.  See pCmdStr for details.

struct callBackStuff {
    int                                 rxFlag;
    enum libusb_transfer_status         rxSts;
    int                                 rxLength;
    uint8_t                             *pRxBuf;

    int                                 txFlag;
    enum libusb_transfer_status         txSts;
    int                                 txLength;
    uint8_t                             *pTxBuf;
} gCallBackInfo;

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

#define VL_PACKET_OVERHEAD	(12)
/* ******************** */


/* ***** Helper routines. ***** */
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


#if 0
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
#endif

void dumpMem(FILE *outFile, uint8_t* pData, int length)
{
    char outStr[DUMP_STR_LENGTH], hexStr[6];
    int  byteIndex, memIndex;
    int  addr;

    byteIndex = memIndex = 0;
    addr = 0;

    /* Setup of the output string. */
    memset(outStr, ' ', DUMP_STR_LENGTH - 1 );
/*
    outStr[DUMP_ASCII_GT] = '>';
    outStr[DUMP_ASCII_LT] = '<';
    outStr[DUMP_STR_LENGTH - 1] = '\0';
*/

    while ( memIndex < length )
    {

        /* Add address to output string */
        if ( 0 == byteIndex )
        {
            sprintf(hexStr, "%04X:", addr & 0xFFFF );
            memcpy(outStr, hexStr, 5);
        }

        /* Add hex byte to output string */
        sprintf( hexStr, "%02X", pData[ memIndex ] );
        if ( 8 > byteIndex )
        {
            memcpy(&outStr[ DUMP_HEX_DATA + (byteIndex * 3) ], hexStr, 2);
        } 
		else 
		{
            /* Extra space after byte 8 */
            memcpy(&outStr[ DUMP_HEX_DATA + 1 + (byteIndex * 3) ], hexStr, 2);
        }

        addr++;
        byteIndex++;
        memIndex++;

        if ( 16 <= byteIndex )
        {
            fprintf( outFile, "%s\n", outStr);

            byteIndex = 0;

            memset( outStr, ' ', DUMP_STR_LENGTH - 1 );
/*
            outStr[ DUMP_ASCII_GT ] = '>';
            outStr[ DUMP_ASCII_LT ] = '<';
            outStr[ DUMP_STR_LENGTH - 1 ] = '\0';
*/
        }
    }   /* while ( memIndex < length ) */

    if ( byteIndex )
    {
        fprintf( outFile, "%s\n", outStr );
    }
    return;
}


/* ****************************************************************************
//	mcan_prt_vl_pkt_header()
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
void mcan_prt_vl_pkt_header(char *pStringHeader, VL_USB_CAN_XFER *pPacket)
{
	// Only print when we should print.
	if (gAPIDebugLevel < DEBUG_1)
		return;

    // Need to keep updated with the number of available commands.
    if ( pPacket->cmd >= 0 && pPacket->cmd <= USB_CAN_CMD_MAX )
    {
        printf("%s Port=%d; Cmd=%s; ", pStringHeader, (int)pPacket->port, 
			pCmdStr[ pPacket->cmd ]);

		// Print the packet ID.
        if ( pPacket->u.txPacket.t0 & CAN_RX_TX_EXTENDED_ID )
        {
            printf("Ext ID: 0x%08X\n", CAN_RX_TX_GET_EXID( pPacket->u.txPacket.t0 ) );
        } else {
            printf("ID: 0x%03X\n", CAN_RX_TX_GET_ID( pPacket->u.txPacket.t0 ) );
        }
	}

    return;
}


/* ****************************************************************************
//	mcan_prt_vl()
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
void mcan_prt_vl(VL_USB_CAN_XFER     *pPacket)
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
        if ( frameSize )
        {
            // NOTE: mcanTxData and mcanRxData will be the same.
            dumpMem( stdout, (uint8_t*) &(pPacket->u.txPacket.mcanTxData), frameSize);
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


/* ****************************************************************************
//	cleanUpUSB()
//	Clean up the USB interface. Usually called after some error condition.
//	
//	Inputs: 
//		None.
//
//	Outputs:
//		None.
//
//	Return Value:
//		None (void).
   ************************************************************************** */
void cleanUpUSB()
{
	int i = 0;
	uint8_t	xfaceIndex;
	int	usbReturnCode;		// USB library routine return values.

	usbReturnCode	= 0;
	xfaceIndex		= 0;
    gRxCancelled	= false;

    libusb_cancel_transfer( pXferBulkIn );
    while ( !gRxCancelled )
    {
        if ( 10 < i++ )
        {
            break;
        }
        usbReturnCode = libusb_handle_events( NULL );
        if ( LIBUSB_SUCCESS != usbReturnCode )
        {
            printf( "\nERROR: RX cancel transfer libusb_handle_events %s\n", libusb_error_name( usbReturnCode ) );
            break;
        }
    }

    if ( NULL != pInBuf ) 
	{ 
		free( pInBuf ); 
	}

    free( gCallBackInfo.pRxBuf );

    for ( xfaceIndex = 0; xfaceIndex < pDevInfo->numInterfaces; xfaceIndex++) 
	{
       libusb_release_interface( pDevInfo->pDevHandle, xfaceIndex );
    }

    for ( xfaceIndex = 0; xfaceIndex < pDevInfo->numInterfaces; xfaceIndex++) 
	{
        libusb_attach_kernel_driver( pDevInfo->pDevHandle, xfaceIndex );
    }

    libusb_reset_device( pDevInfo->pDevHandle );
    libusb_close( pDevInfo->pDevHandle );

    libusb_exit( pUsbContext );
    return;
}


// Callback function for the USB bulkin port.
void rxCallBack(struct libusb_transfer *pXfer)
{
	/* ***** Declarations. ***** */
	VL_USB_CAN_XFER	*pRxPacket;
	/* ************************* */

	// We are here.
	if (gAPIDebugLevel >= DEBUG_2)
		printf("Entering rxCallBack()\n");

	gRxPacketCount++;
    if ( LIBUSB_TRANSFER_COMPLETED == pXfer->status )
    {
		if (gAPIDebugLevel >= DEBUG_2)
			printf("Status complete;\n");

        gCallBackInfo.rxFlag = 1;
        gCallBackInfo.rxSts = (int) pXfer->flags;
        //gCallBackInfo.rxLength = (int) pXfer->actual_length;
        //memcpy(gCallBackInfo.pRxBuf, pXfer->buffer, pXfer->actual_length);
        gCallBackInfo.rxLength = (int) (pXfer->actual_length + sizeof(MCAN_RX_PACKET));
        memcpy(gCallBackInfo.pRxBuf, pXfer->buffer, (pXfer->actual_length + sizeof(MCAN_RX_PACKET)));
		gRxTimeOutCount = 0;

		// For ease of reference, create a CAN packet.
		pRxPacket = (VL_USB_CAN_XFER *)gCallBackInfo.pRxBuf;
		if (gAPIDebugLevel >= DEBUG_2)
		{
			mcan_prt_vl_pkt_header("rxCallBack():", pRxPacket);
		}

		// Filter out certain packets.
		switch(pRxPacket->cmd)
		{
			case USB_CAN_CMD_OPEN:
			case USB_CAN_CMD_CLOSE:
			case USB_CAN_CMD_TX:
			case USB_CAN_CMD_ID:
			case USB_CAN_CMD_MAX:
			case USB_CAN_CMD_UNDEFINED: 
				break;  // Do nothing for these.
			case USB_CAN_CMD_STS:
				gCANPortStatus = pRxPacket->misc;
				break;
			case USB_CAN_CMD_MCU_FW_VER: 
				gMCUFwReceived = pRxPacket->extendedStatus;
				break;
			case USB_CAN_CMD_RX:
				if (gAPIDebugLevel >= DEBUG_1)
				{
					mcan_prt_vl(pRxPacket);
				}
				break;
			default:
				break;
		}
    } else if ( LIBUSB_TRANSFER_TIMED_OUT == pXfer->status ) {
		if (gAPIDebugLevel >= DEBUG_1)
		{
			if (gAPIDebugLevel >= DEBUG_2)
				printf("Status timed_out;\n");
		}
        gRxTimeOutExit = true;
		gRxTimeOutCount ++;
    } else if ( LIBUSB_TRANSFER_CANCELLED == pXfer->status ) {
		if (gAPIDebugLevel >= DEBUG_2)
			printf("Status cancelled;\n");
        gRxCancelled = true;
		gRxTimeOutCount = 0;
    }
	else
	{
		if (gAPIDebugLevel >= DEBUG_2)
			printf("UNKNOWN STATUS (%d);\n", pXfer->status);
	}

    libusb_submit_transfer( pXfer );

	if (gAPIDebugLevel >= DEBUG_2)
		printf("Leaving rxCallBack(rxPacketCount=%d; rxTimeOutCount=%d);\n",
			gRxPacketCount, gRxTimeOutCount);

    return;
}


// Callback function for the USB bulkout port.
void txCallBack( struct libusb_transfer      *pXfer)
{
	if (gAPIDebugLevel >= DEBUG_2)
    	printf("Entering txCallBack() - gTxCancelled=%d; gAboutFlag=%d\n",
			gTxCancelled, gAbortFlag);

    if ( LIBUSB_TRANSFER_COMPLETED == pXfer->status )
    {
        gCallBackInfo.txFlag = 1;
        gCallBackInfo.txSts = (int) pXfer->flags;

    } else if ( LIBUSB_TRANSFER_CANCELLED == pXfer->status ) {
        gTxCancelled = true;

    } else {
        gAbortFlag = true;
    }

	if (gAPIDebugLevel >= DEBUG_2)
    	printf("Leaving txCallBack() - gTxCancelled=%d; gAboutFlag=%d\n",
			gTxCancelled, gAbortFlag);

    return;
}


/* ****************************************************************************
//	sendFrame()
//	Sends a packet out the specified USB port.
//
//	Inputs:
//		None.
//
//	Globals:
//		libusb_transfer	*pUsbContext	=> Data to go out.
//
//	Outputs:
//		None.
//
//	ReturnValue:
//		TRUE	=> Data sent without error.
//		FALSE	=> Error sending data.
   ************************************************************************** */
bool sendFrame()
{
    bool        rtn = true;
    int         usbSts;

	if (gAPIDebugLevel >= DEBUG_2)
		printf("Entering sendFrame();\n");

	// Send the data out.
    usbSts = libusb_submit_transfer( pXferBulkOut );

    while ( !gAbortFlag )
    {
        usbSts = libusb_handle_events( NULL );
        if ( LIBUSB_SUCCESS != usbSts )
        {
			if (gAPIDebugLevel >= DEBUG_2)
			{
            	printf( "\nERROR: sendFrame libusb_handle_events %s\n", 
					libusb_error_name( usbSts ) );
			}
            rtn = false;
            break;
        }

        if ( gCallBackInfo.txFlag )
        {
            gCallBackInfo.txFlag = 0;
            break;
        }
    }

	// Done.
	if (gAPIDebugLevel >= DEBUG_2)
		printf("Leaving sendFrame;\n");

    return( rtn );
}

/* ****************************************************************************
//	sendFrameRcvFrame()
//	Sends a packet out the specified USB port.
//
//	Inputs:
//		libusb_transfer	*pUsbContext	=> Data to go out.
//
//	Outputs:
//		None.
//
//	ReturnValue:
//		TRUE	=> Data sent without error.
//		FALSE	=> Error sending data.
   ************************************************************************** */
bool sendFrameRcvFrame()
{
    bool        rtn = true;
    int         usbSts;

	if (gAPIDebugLevel >= DEBUG_1)
		printf("Entering sendFrameRcvFrame();\n");

	// Send the data out.
    usbSts = libusb_submit_transfer(pXferBulkOut);

    while ( !gAbortFlag )
    {
        usbSts = libusb_handle_events( NULL );
        if ( LIBUSB_SUCCESS != usbSts )
        {
            printf( "\nERROR: sendFrameRcvFrame libusb_handle_events %s\n", 
				libusb_error_name( usbSts ) );
            rtn = false;
            break;
        }

        if ( gCallBackInfo.txFlag )
        {
            gCallBackInfo.txFlag = 0;
            break;
        }

        if ( gCallBackInfo.rxFlag )
        {
            gCallBackInfo.rxFlag = 0;
            break;
        }
    }

	// Done.
	if (gAPIDebugLevel >= DEBUG_1)
		printf("Leaving sendFrameRcvFrame;\n");

    return( rtn );
}


/* ****************************************************************************
//  USBDeviceOpen()
//  Creates a handle to a CAN port. Used in many other CAN calls.
//
//  Inputs:
//		VL_CANBoardT 		boardName	=>	Name of the CAN board to open.  This 
//											allow
//											for multiple CAN cards on one carrier.
//											VL_CAN_BOARD0, VL_CAN_BOARD1.
//		VL_CANPortT 		portName	=>	Name of the CAN port to open. 
//
//	Outputs:
//		None 
//
//	Globals modified:
//		gUSBDeviceOpened
//		gPort0Opened
//		gPort1Opened
//		pXferBulkOut
//		pXferBulkIn
//
//  Return values:
//		VL_API_OK						=> Successfully opened the CAN port
//		VL_API_INVALID_ARG				=> Invalid input passed in.
//		VL_API_CAN_OPEN_DEVICE_ERROR	=> Invalid input passed in.
//		VL_API_CAN_OPEN_NO_DEVICE_FOUND	=> No CAN devices found on carrier board.
//		
   ************************************************************************** */
CmdStatusT USBDeviceOpen(VL_CANBoardT  boardName, VL_CANPortT   portName )
{
	/* ***** Declarations. ***** */
	CmdStatusT	returnCode;
	int				usbReturnCode;		// USB library routine return values.

	libusb_device	**devList;			// List of USB devices.
	libusb_device	*pDevFound;			// USB device found in list.
	ssize_t			devCount;			// USB device count.
	int				numDevices;			// Number of USB devices fount.
	int				devIndex;			// Index into list of USB devices.
	int				usbChannelIndex;	// Index to the USB channels.
	uint8_t			configIndex, libUsbDevIndex, xfaceIndex, epIndex;	// Various indexes.
	
	struct libusb_device_descriptor				devDescriptor;
    const struct libusb_interface_descriptor	*pXfaceDescriptor;
    const struct libusb_interface				*pInterface;

	libusb_device_handle 						*pDevHandle;
	/* ************************* */
	
	/* ***** Where are we? ***** */
	if (gAPIDebugLevel >= DEBUG_1)
		printf("\tEntering USBDeviceOpen();\n");
	/* ************************** */

	/* ***** Parameter error checking. ***** */
	// Board name.
	if ((boardName < VL_CAN_BOARD0) || (boardName > VL_CAN_BOARD1))
	{
		// Invalid CAN board specified.
		return(VL_API_INVALID_ARG);
	}

	// Port name.
	if ((portName < VL_CAN_PORT0) || (portName > VL_CAN_PORT1))
	{
		// Invalid CAN board specified.
		return(VL_API_INVALID_ARG);
	}
	/* ************************************* */

	/* ***** Initializations. ***** */
	returnCode	= VL_API_OK;  // Assume all is OK.
	/* **************************** */

	/* ***** Get a list of USB devices. */
	/* Initialize the USB subsystem. */
	usbReturnCode = libusb_init(&pUsbContext);
	if (usbReturnCode)
	{
		// ERROR in opening the USB context.
		if (gAPIDebugLevel >= DEBUG_1)
		{
			printf("\tUSBDeviceOpen: Fail to initialize USB interface: %d:%d, %s;\n",
				boardName, portName, libusb_error_name(usbReturnCode));
		}

		// USB device is NOT ready to use.
		gUSBDeviceOpened = false;

		return(VL_API_CAN_OPEN_DEVICE_ERROR);
	}

	/* Initial USB interfaces ok. */
	/* Load device list to discover devices. */
	devCount = libusb_get_device_list(NULL, &devList);
	if (0 > devCount)
	{
		/* Problem, no USB devices found. */
		// Clean up after ourselves.
		libusb_exit(pUsbContext);
		if (gAPIDebugLevel >= DEBUG_1)
		{
			printf("\tUSBDeviceOpen: Fail to find USB interface: %d:%d, %s;\n",
				boardName, portName, libusb_error_name(usbReturnCode));
		}

		// USB device is NOT ready to use.
		gUSBDeviceOpened = false;

		return(VL_API_CAN_OPEN_NO_DEVICE_FOUND);
	}

	/* USB devices found. */
	/* Scan the device list looking for devices that have proper vendor and 
	   product IDs. Build array for devices and device descriptors. */
    for(numDevices = devIndex = 0; 
		(devIndex < devCount) && (MAX_NUM_USB_DEVICES > numDevices);
		devIndex++)
    {
		// Get the current device.
        pDevFound = devList[devIndex];

		// Get the current device's device descriptor.
        usbReturnCode = libusb_get_device_descriptor(pDevFound, &devDescriptor);
        if (usbReturnCode)
        {
			// Clean up. 
            libusb_free_device_list(devList, 1);
            libusb_exit(pUsbContext);

			if (gAPIDebugLevel >= DEBUG_1)
			{
				printf("\tUSBDeviceOpen: Fail to retrieve USB descriptor: %d:%d, %s;\n",
					boardName, portName, libusb_error_name(usbReturnCode));
			}

			// USB device is NOT ready to use.
			gUSBDeviceOpened = false;

			return(VL_API_CAN_OPEN_NO_DEVICE_DESC);
        }

		// Have a valid device descriptor.
		// Now look for our Vendor and Product IDs.
        if ((OUR_VENDOR == devDescriptor.idVendor) && 
			(OUR_PRODUCT == devDescriptor.idProduct))
        {
            pDevicesFound[numDevices] = pDevFound;
            numDevices++;
        }
    } // for (numDevices ...)

	/* Clean up. */
    libusb_free_device_list(devList, 1);

	/* Did we sucessfully find USB device(s) that match our vendor and product 
	   ids and have ports? */
	if (numDevices == 0)
	{
		// No matching devices found.
		if (gAPIDebugLevel >= DEBUG_1)
		{
			printf("\tUSBDeviceOpen: Fail to find our USB device: %d:%d, %s;\n",
				boardName, portName, libusb_error_name(usbReturnCode));
		}

		// USB device is NOT ready to use.
		gUSBDeviceOpened = false;

		return(VL_API_CAN_OPEN_NO_DEVICE_FOUND);
	}
	else
	{
		/* Found at least 1 matching device. */
		if (gAPIDebugLevel >= DEBUG_1)
		{
			printf("Board %d: Found %d CAN device(s).\n", boardName, numDevices);
		}
	}
	/* ******************************* */

	/* ***** Gather endpoint information on VL USB channels. ***** */
    for (devIndex = 0; devIndex < numDevices; devIndex++)
    {
        struct libusb_config_descriptor     *pConfigDescriptor;

        usbReturnCode = libusb_get_device_descriptor(pDevicesFound[devIndex], 
							&devDescriptor );
        if (usbReturnCode)
        {
			// Error getting device descriptor. Cleanup.
            libusb_exit(pUsbContext);

            if (gAPIDebugLevel >= DEBUG_1)
			{
				printf("Board %d: Interface %d, failed to get device descriptor.\n",
					boardName, portName);
            	libusb_error_name(usbReturnCode);
            }

			// USB device is NOT ready to use.
			gUSBDeviceOpened = false;

			return(VL_API_CAN_OPEN_NO_DEVICE_DESC);
        }
		else
		{
			if (gAPIDebugLevel >= DEBUG_1)
			{
				printf("Board %d: Interface %d, found device descriptor.\n",
					boardName, portName);
			}

			if (gAPIDebugLevel >= DEBUG_2)
			{
				printf("\t\tdevIndex=%d;\n", devIndex);
			}
		}

        configIndex = 0;

        // Endpoint addresses may be the same for multiple cards.
        // So keep device information to use to figure which is which.
        devicesInfoEpAddrs[devIndex].devDescriptor = devDescriptor;

        // Use to open device with libusb_open.
        devicesInfoEpAddrs[devIndex].pLibUsbDev = pDevicesFound[devIndex];

        // Set when libusb_open is called with .libUsbDev.
        devicesInfoEpAddrs[devIndex].pDevHandle = (libusb_device_handle*) NULL;

        usbReturnCode = libusb_get_config_descriptor(
							devicesInfoEpAddrs[ devIndex ].pLibUsbDev, configIndex,
          					&pConfigDescriptor);
        if (usbReturnCode)
        {
			// Error getting configuration descriptor. Cleanup.
            libusb_exit(pUsbContext);

            if (gAPIDebugLevel >= DEBUG_1)
			{
				printf("Board %d: Interface %d, failed to get configuration descriptor.\n",
					boardName, portName);
            	libusb_error_name(usbReturnCode);
            }	

			// USB device is NOT ready to use.
			gUSBDeviceOpened = false;

			return(VL_API_CAN_OPEN_NO_DEVICE_DESC);
        }
		else
		{
			if (gAPIDebugLevel >= DEBUG_1)
			{
				printf("Board %d: Interface %d, found config descriptor.\n",
					boardName, portName);
			}

			if (gAPIDebugLevel >= DEBUG_2)
			{
				printf("\t\tdevIndex=%d;\n", devIndex);
			}
		}

        devicesInfoEpAddrs[devIndex].numInterfaces = (int) pConfigDescriptor->bNumInterfaces;

        usbChannelIndex = -1;
        for (libUsbDevIndex = 0; libUsbDevIndex < devicesInfoEpAddrs[devIndex].numInterfaces;
          		libUsbDevIndex++) 
		{
            pInterface = &(pConfigDescriptor->interface[libUsbDevIndex]);
            for (xfaceIndex = 0; xfaceIndex < pInterface->num_altsetting; xfaceIndex++) 
			{
                pXfaceDescriptor = &(pInterface->altsetting[xfaceIndex]);
                for (epIndex = 0; epIndex < pXfaceDescriptor->bNumEndpoints; epIndex++)
                {
                    if (USB_CLASS_CDC_MODEM == pXfaceDescriptor->bInterfaceClass)
                    {
                        usbChannelIndex++;
                        devicesInfoEpAddrs[devIndex].epAddrs[usbChannelIndex].control = 
                        	pXfaceDescriptor->endpoint[epIndex].bEndpointAddress;
                    } else 
					{
                        if (0x80 & pXfaceDescriptor->endpoint[epIndex].bEndpointAddress)
                        {
                            devicesInfoEpAddrs[devIndex].epAddrs[usbChannelIndex].bulkIn = 
                            	pXfaceDescriptor->endpoint[epIndex].bEndpointAddress;
							if (gAPIDebugLevel >= DEBUG_2)
							{
								printf("\t\tbulkin EP at epIndex %d; address 0x%x;\n",
									epIndex, 
									pXfaceDescriptor->endpoint[epIndex].bEndpointAddress);
							}
                        } else 
						{
                            devicesInfoEpAddrs[devIndex].epAddrs[usbChannelIndex].bulkOut = 
                            	pXfaceDescriptor->endpoint[epIndex].bEndpointAddress;
							if (gAPIDebugLevel >= DEBUG_2)
							{
								printf("\t\tbulkout EP at epIndex %d; address 0x%x;\n",
									epIndex, 
									pXfaceDescriptor->endpoint[epIndex].bEndpointAddress);
							}
                        }
                    }
                }  // for epIndex
            }  // for xfaceIndex
        }  // for xface
    }  // for devIndex
	/* ************************************************************ */
	
	/* ***** Now claim the interfaces for our use. ***** */
	pDevInfo = &(devicesInfoEpAddrs[boardName]);

	// Open the USB device.
    usbReturnCode = libusb_open(pDevInfo->pLibUsbDev, &pDevHandle);
    if (usbReturnCode)
    {
		// Error opening the device. Clean up. */
        libusb_exit(pUsbContext);

        if (gAPIDebugLevel >= DEBUG_1)
		{
			printf("Board %d: Unable to open USB device.\n", boardName);
        	printf("\t\tError details:%s;\n", libusb_error_name(usbReturnCode));
        }

		// USB device is NOT ready to use.
		gUSBDeviceOpened = false;

		return(VL_API_CAN_OPEN_DEVICE_ERROR);
	}

	// Save the current devices handle.
    pDevInfo->pDevHandle = pDevHandle;
    gpDevHandle = pDevHandle;


    // This fails since there is only 1 configuration and it is already used.
    libusb_reset_device(pDevInfo->pDevHandle);

    usbReturnCode = libusb_set_configuration(pDevInfo->pDevHandle, 0);

	/* ***** Claim the interface for our use. ***** */
    for ( xfaceIndex = 0; xfaceIndex < pDevInfo->numInterfaces; xfaceIndex++) 
	{
        usbReturnCode = libusb_kernel_driver_active(pDevInfo->pDevHandle, xfaceIndex);
        if (1 == usbReturnCode)
        {
			// There is an active kernel driver attached to this port, 
			// deattach it so we can use it.
            usbReturnCode = libusb_detach_kernel_driver(pDevInfo->pDevHandle, xfaceIndex);
            if (usbReturnCode)
            {
                libusb_close( pDevInfo->pDevHandle );
                libusb_exit( pUsbContext );

        		if (gAPIDebugLevel >= DEBUG_1)
				{
					printf(
						"Board %d: Interface %d, unable to detach kernel driver for xface %d %s\n\n",
						boardName, portName, xfaceIndex, libusb_error_name(usbReturnCode));
        		}

				// USB device is NOT ready to use.
				gUSBDeviceOpened = false;

				return(VL_API_CAN_OPEN_DEVICE_ERROR);
            }
        } 
		else if (LIBUSB_ERROR_NO_DEVICE == usbReturnCode)
		{
            libusb_close( pDevInfo->pDevHandle );
            libusb_exit( pUsbContext );

        	if (gAPIDebugLevel >= DEBUG_1)
			{
				printf(
					"Board %d: Interface %d, Device USB Error xface %d %s\n\n",
					boardName, portName, xfaceIndex, libusb_error_name(usbReturnCode));
        	}

			// USB device is NOT ready to use.
			gUSBDeviceOpened = false;

			return(VL_API_CAN_OPEN_DEVICE_ERROR);
		}
		else  // 0 was returned => no driver is active on the interface.
		{
			// This is OK, just report the condition.
        	if (gAPIDebugLevel >= DEBUG_2)
			{
				printf("Board %d: Interface %d, kernel driver not active for xface %d.\n",
					boardName, portName, xfaceIndex);
        	}
        }

		// Now claim the actual interface. 
        usbReturnCode = libusb_claim_interface(pDevInfo->pDevHandle, xfaceIndex);
        if (usbReturnCode)
        {
            libusb_close( pDevInfo->pDevHandle );
            libusb_exit( pUsbContext );

        	if (gAPIDebugLevel >= DEBUG_1)
			{
				printf(
					"Board %d: Interface %d, unable to claim interface %d %s\n\n",
					boardName, portName, xfaceIndex, libusb_error_name(usbReturnCode));
        	}

			// USB device is NOT ready to use.
			gUSBDeviceOpened = false;

			return(VL_API_CAN_OPEN_DEVICE_ERROR);
        }

        if (gAPIDebugLevel >= DEBUG_1)
		{
			printf( "INFO: Board %d: Interface %d, claimed interface %d %s\n\n",
				boardName, portName, xfaceIndex, libusb_error_name(usbReturnCode));
        }

		// USB device is ready to use.
		gUSBDeviceOpened = true;

    }  // for (xfaceIndex = 0, ...)
	/* ******************************************** */

	/* ***** Setup USB for async operations. ***** */
	pXferBulkIn	 = libusb_alloc_transfer(0);
	pXferBulkOut = libusb_alloc_transfer(0);
	/* ******************************************* */

	/* ***** Done. ***** */
	if (gAPIDebugLevel >= DEBUG_1)
		printf("\tLeaving USBDeviceOpen(): %d;\n", returnCode);
	return(returnCode);
	/* ***************** */
}


/* ********** Exposed Functions. ********** */
/* ***** CAN port routines. ***** */
/* ****************************************************************************
//  ex_CanOpenPort()
//  Creates a handle to a CAN port. Used in many other CAN calls.
//
//  Inputs:
//		VL_CANBoardT 		boardName	=>	Name of the CAN board to open.  This 
//											allows for multiple CAN cards on one 
//											carrier. VL_CAN_BOARD0, VL_CAN_BOARD1.
//		VL_CANPortT 		portName	=>	Name of the CAN port to open. 
//											VL_CAN_PORT0, VL_CAN_PORT1.
//
//		uint32_t	arbitrationBitRate	=>	Arbitration bit rate (bits/second)
//
//		uint32_t	dataBitRate			=>	Data bit rate (bits/second)- 
//											Must be <= arbitrationBitRate.
//
//		uint32_t	rxEnabled			=> Receive packet after transmit
//										   TRUE or FALSE. Default is TRUE.
//
//		uint32_t	loopbackEnabled		=> Loopback enabled. TRUE or FALSE.
//
//	Outputs:
//		None 
//
//  Return values:
//		VL_API_OK						=> Successfully opened the CAN port
//		VL_API_INVALID_ARG				=> Invalid input passed in.
//		VL_API_CAN_OPEN_DEVICE_ERROR	=> Invalid input passed in.
//		VL_API_CAN_OPEN_NO_DEVICE_FOUND	=> No CAN devices found on carrier board.
//		
//
//  The first CANOpenPort() to a CAN port initializes the hardware to default
//  parameter 500 kbit/sec and acceptance of extended frames.
   ************************************************************************** */
CmdStatusT ex_CanOpenPort(	VL_CANBoardT  boardName, 
						  	VL_CANPortT   portName, 
							uint32_t      arbitrationBitRate,
							uint32_t      dataBitRate,
							uint32_t	  rxEnabled,
							uint32_t      loopbackEnabled)
{
	/* ***** Declarations. ***** */
	CmdStatusT	returnCode;
	bool			bRtn;
	/* ************************* */
	
	/* ***** Where are we? ***** */
	if (gAPIDebugLevel >= DEBUG_1)
		printf("Entering ex_CanOpenPort(board=%d, port=%d, deviceReady=%d);\n",
			boardName, portName, gUSBDeviceOpened);
	/* ************************** */

	/* ***** Parameter error checking. ***** */
	// Board name.
	if ((boardName < VL_CAN_BOARD0) || (boardName > VL_CAN_BOARD1))
	{
		// Invalid CAN board specified.
		return(VL_API_INVALID_ARG);
	}

	// Port name.
	if ((portName < VL_CAN_PORT0) || (portName > VL_CAN_PORT1))
	{
		// Invalid CAN board specified.
		return(VL_API_INVALID_ARG);
	}

	if ((loopbackEnabled < 0) || (loopbackEnabled > 1))
	{
		// loopbackEnabled out of bounds.
		if (gAPIDebugLevel >= DEBUG_1)
		{
			printf("\tex_CanOpenPort: loopbackEnable specified is invalid: %d\n", 
				loopbackEnabled);
		}
		return(VL_API_INVALID_ARG);
	}
	/* ************************************* */

	/* ***** Open the USB device if needed. ***** */
	returnCode = VL_API_OK;  // Assume all is OK.

	// Is the USB device ready to use.
	if (true == gUSBDeviceOpened)
	{
		if (gAPIDebugLevel >= DEBUG_1)
		{
			printf("\tUSB-CAN interface running;\n");
		}
	}
	else
	{
		if (gAPIDebugLevel >= DEBUG_1)
		{
			printf("\tUSB-CAN interface being initialized;\n");
		}

		returnCode = USBDeviceOpen(boardName, portName);
		if (returnCode != VL_API_OK)
		{
			// Error, could not open the USB device connection to the MCU.
			if (gAPIDebugLevel >= DEBUG_1)
			{
				printf("\tex_CanOpenPort: Could not open USB connection to MCU: Error code %d\n",
					returnCode);
			}
			
			return(VL_API_CAN_OPEN_DEVICE_ERROR);
		}
		else
		{
			if (gAPIDebugLevel >= DEBUG_1)
			{
				printf("\tUSB-CAN interface initialized;\n");
			}
		}
	}
	/* ****************************************** */

	/* ***** Send the Open command to the MCU. ***** */
	// Setup the callback info.
	gCallBackInfo.rxFlag = gCallBackInfo.txFlag = 0;
	gCallBackInfo.rxSts  = gCallBackInfo.txSts  = 0;
	gCallBackInfo.rxLength = 0;
	gCallBackInfo.txLength = 0;
	gCallBackInfo.pRxBuf = malloc( RX_BUF_SIZE );


	// Setup the command.
	canCmd.size							= USB_CAN_CMD_OPEN_SIZE;
	canCmd.cmd							= USB_CAN_CMD_OPEN;
	canCmd.u.openInfo.arbitrationRate	= arbitrationBitRate;
    canCmd.u.openInfo.dataRate			= dataBitRate;
    canCmd.u.openInfo.loopback			= loopbackEnabled;

	bulkOut	= pDevInfo->epAddrs[0].bulkOut;
	bulkIn	= pDevInfo->epAddrs[0].bulkIn;

	pXferBulkOut->buffer			= (uint8_t*) &canCmd;
	pXferBulkOut->length			= canCmd.size;
	pXferBulkOut->actual_length		= canCmd.size;

	// Setup transfer structs for async operation.
	// Bulk in
    pInBuf = malloc( RX_BUF_SIZE );
    libusb_fill_bulk_transfer(
        pXferBulkIn,
        pDevInfo->pDevHandle,
        bulkIn,
        pInBuf,
        RX_BUF_SIZE,
        rxCallBack,
        NULL,
        100);

    pXferBulkIn->flags &= ~(LIBUSB_TRANSFER_FREE_BUFFER | LIBUSB_TRANSFER_FREE_TRANSFER);
    libusb_submit_transfer( pXferBulkIn );

	// Bulk out
    libusb_fill_bulk_transfer( pXferBulkOut, pDevInfo->pDevHandle, bulkOut,
        					  (uint8_t*) NULL, 0, txCallBack, NULL, 1000);

    pXferBulkOut->flags &= ~(LIBUSB_TRANSFER_FREE_BUFFER | LIBUSB_TRANSFER_FREE_TRANSFER);	

	// Send the Open packet.
	canCmd.port							= (uint8_t)portName;
	canCmd.u.openInfo.arbitrationRate	= arbitrationBitRate;
    canCmd.u.openInfo.dataRate			= dataBitRate;
    canCmd.u.openInfo.loopback			= loopbackEnabled;

	pXferBulkOut->buffer			= (uint8_t*) &canCmd;
	pXferBulkOut->length			= canCmd.size;
	pXferBulkOut->actual_length		= canCmd.size;

	bRtn = sendFrame();

	// Check for successful open
	// sendFrame(): returns 0 on sucess, else error code.
	if (!bRtn)
	{
		// Error on sending the port open command.
		if(gAPIDebugLevel >= DEBUG_1)
		{
			printf("\tex_CanOpenPort: Error opening CAN Port: board %d, port %d, error code %d\n",
				boardName, portName, bRtn);
		}

		cleanUpUSB();

		if(0 == portName)
			gPort0Opened = false;
		else
			gPort1Opened = false;
		
		return(VL_API_CAN_OPEN_PORT_ERROR);
	}
	else
	{
		if(0 == portName)
			gPort0Opened = true;
		else
			gPort1Opened = true;
	}

	// If debug output in MCU is used do this.
    usleep( 1000000 );

	// Send the rxXfer packet to turn on/off sending the transmitted packet
	// back.
	canCmd.port	= portName;
	canCmd.cmd	= USB_CAN_CMD_RX_CTRL;
	canCmd.size	= USB_CAN_CMD_RX_CTRL_SIZE;
	canCmd.misc	= rxEnabled; 
	
	pXferBulkOut->buffer		= (uint8_t*) &canCmd;
	pXferBulkOut->length		= canCmd.size;
	pXferBulkOut->actual_length	= canCmd.size;

	bRtn = sendFrame();

	// Check for success.
	// sendFrame(): returns 0 on sucess, else error code.
	if (!bRtn)
	{
		// Error on sending the port open command.
		if(gAPIDebugLevel >= DEBUG_1)
		{
			printf("\tex_CanOpenPort: turning off packet receive on transmit: board %d, port %d, error code %d\n",
				boardName, portName, bRtn);
		}

		cleanUpUSB();

		if(0 == portName)
			gPort0Opened = false;
		else
			gPort1Opened = false;
		
		return(VL_API_CAN_OPEN_PORT_ERROR);
	}
	/* ********************************************* */

	/* ***** Done. ***** */
	if (gAPIDebugLevel >= DEBUG_1)
		printf("Leaving ex_CanOpenPort();\n");
	return(returnCode);
	/* ***************** */
}


/* ****************************************************************************
//  ex_CanOpenPortWithUserTiming()
//  Creates a handle to a CAN port. Used in many other CAN calls.
//
//  Inputs:
//		VL_CANBoardT boardName	=>	Name of the CAN board to open.  This 
//											allows for multiple CAN cards on one 
//											carrier. VL_CAN_BOARD0, VL_CAN_BOARD1.
//		VL_CANPortT  portName	=>	Name of the CAN port to open. 
//											VL_CAN_PORT0, VL_CAN_PORT1.
//
//		uint32_t	arbitrationBitRate	=>	Arbitration bit rate (bits/second)
//
//		uint32_t	dataBitRate			=>	Data bit rate (bits/second)- 
//											Must be <= arbitrationBitRate.
//
//		uint8_t	    rxEnabled  			=> Receive packet after transmit
//										   TRUE or FALSE. Default is TRUE.
//
//		uint8_t	     loopbackEnabled	=> Loopback enabled. TRUE or FALSE.
//		  
//		uint32_t	 masterDivider		 =>	Master divider
//
//		uint16_t     arbPreDivider       =>  Arbitration clock pre-scalar
//		                                     division factor.
//
//		uint8_t      arbResyncJumpWidth  =>  Re-synce Jump Width.
//
//		uint8_t      arbSeg1             => Data Time Segment 1.
//
//		uint_8       arbSeg2             => Data Time Segment 2.
//
//		uint16_t     dataPreDivider      =>  Arbitration clock pre-scalar
//		                                    division factor.
//
//		uint8_t      dataResyncJumpWidth =>  Re-synce Jump Width.
//
//		uint8_t      dataSeg1            => Data Time Segment 1.
//
//		uint_8       dataSeg2            => Data Time Segment 2.
//
//	Outputs:
//		None 
//
//  Return values:
//		VL_API_OK						=> Successfully opened the CAN port
//		VL_API_INVALID_ARG				=> Invalid input passed in.
//		VL_API_CAN_OPEN_DEVICE_ERROR	=> Invalid input passed in.
//		VL_API_CAN_OPEN_NO_DEVICE_FOUND	=> No CAN devices found on carrier board.
//		
//
//  The first CANOpenPort() to a CAN port initializes the hardware to default
//  parameter 500 kbit/sec and acceptance of extended frames.
   ************************************************************************** */
CmdStatusT ex_CanOpenPortWithUserTiming(VL_CANBoardT boardName, 
										VL_CANPortT  portName, 
										uint32_t     arbitrationBitRate,
										uint32_t     dataBitRate,
										uint8_t	     rxEnabled,
										uint8_t      loopbackEnabled,
                                        uint32_t	 masterDivider,
                                        uint16_t     arbPreDivider,
                                        uint8_t      arbResyncJumpWidth,
                                        uint8_t      arbSeg1,
                                        uint8_t      arbSeg2,
                                        uint16_t     dataPreDivider,
                                        uint8_t      dataResyncJumpWidth,
                                        uint8_t      dataSeg1,
                                        uint8_t      dataSeg2)
{
	/* ***** Declarations. ***** */
	CmdStatusT	returnCode;
	bool		bRtn;
	/* ************************* */
	
	/* ***** Where are we? ***** */
	if (gAPIDebugLevel >= DEBUG_1)
	{
		printf("Entering ex_CanOpenPortWithUserTiming(board=%d, port=%d, deviceReady=%d);\n",
			boardName, portName, gUSBDeviceOpened);
	}
	/* ************************** */
	printf("\n\tARate=%d; DRate=%d\n", arbitrationBitRate, dataBitRate);
	printf("\trxEnabled=%d; loopbackEnabled=%d\n", rxEnabled, loopbackEnabled);
	printf("\tmasterDivider=%d\n", masterDivider);
	printf("\tAPreDivider=%d; ARSync=%d; AS1=%d; AS2=%d\n", arbPreDivider, arbResyncJumpWidth,
	                                                        arbSeg1, arbSeg2);
	printf("\tDPreDivider=%d; DRSync=%d; DS1=%d; DS2=%d\n", dataPreDivider, dataResyncJumpWidth,
					                                        dataSeg1, dataSeg2);
	printf("\tcommand=%d; size=%d\n", USB_CAN_CMD_OPEN_USR_TIMING, 
					                  USB_CAN_CMD_OPEN_USR_TIMING_SIZE);

	/* ***** Parameter error checking. ***** */
	// Board name.
	if ((boardName < VL_CAN_BOARD0) || (boardName > VL_CAN_BOARD1))
	{
		// Invalid CAN board specified.
		return(VL_API_INVALID_ARG);
	}

	// Port name.
	if ((portName < VL_CAN_PORT0) || (portName > VL_CAN_PORT1))
	{
		// Invalid CAN board specified.
		return(VL_API_INVALID_ARG);
	}

	if ((loopbackEnabled < 0) || (loopbackEnabled > 1))
	{
		// loopbackEnabled out of bounds.
		if (gAPIDebugLevel >= DEBUG_1)
		{
			printf("\tex_CanOpenPortWithUserTiming: loopbackEnable specified is invalid: %d\n", 
				loopbackEnabled);
		}
		return(VL_API_INVALID_ARG);
	}
	/* ************************************* */

	/* ***** Fill in the timing structure. ***** */
	// Main pieces.
	canCmd.u.openInfo.masterDivider  = masterDivider;

	// Arbitration pieces.
	canCmd.u.openInfo.arbPreDivider = arbPreDivider;
	canCmd.u.openInfo.arbRJumpwidth = arbResyncJumpWidth;
	canCmd.u.openInfo.arbSeg1       = arbSeg1;
	canCmd.u.openInfo.arbSeg2       = arbSeg2;
	
	// Data pieces.
	canCmd.u.openInfo.dataPreDivider = dataPreDivider;
	canCmd.u.openInfo.dataRJumpwidth = dataResyncJumpWidth;
	canCmd.u.openInfo.dataSeg1       = dataSeg1;
	canCmd.u.openInfo.dataSeg2       = dataSeg2;
	/* ***************************************** */

	/* ***** Open the USB device if needed. ***** */
	returnCode = VL_API_OK;  // Assume all is OK.

	// Is the USB device ready to use.
	if (true == gUSBDeviceOpened)
	{
		if (gAPIDebugLevel >= DEBUG_1)
		{
			printf("\tUSB-CAN interface running;\n");
		}
	}
	else
	{
		if (gAPIDebugLevel >= DEBUG_1)
		{
			printf("\tUSB-CAN interface being initialized;\n");
		}

		returnCode = USBDeviceOpen(boardName, portName);
		if (returnCode != VL_API_OK)
		{
			// Error, could not open the USB device connection to the MCU.
			if (gAPIDebugLevel >= DEBUG_1)
			{
				printf("\tex_CanOpenPortWithUserTiming: Could not open USB connection to MCU: Error code %d\n",
					returnCode);
			}
			
			return(VL_API_CAN_OPEN_DEVICE_ERROR);
		}
		else
		{
			if (gAPIDebugLevel >= DEBUG_1)
			{
				printf("\tUSB-CAN interface initialized;\n");
			}
		}
	}
	/* ****************************************** */

	/* ***** Send the Open command to the MCU. ***** */
	// Setup the callback info.
	gCallBackInfo.rxFlag = gCallBackInfo.txFlag = 0;
	gCallBackInfo.rxSts  = gCallBackInfo.txSts  = 0;
	gCallBackInfo.rxLength = 0;
	gCallBackInfo.txLength = 0;
	gCallBackInfo.pRxBuf = malloc( RX_BUF_SIZE );


	// Setup the command.
	canCmd.size							= USB_CAN_CMD_OPEN_USR_TIMING_SIZE;
	canCmd.cmd							= USB_CAN_CMD_OPEN_USR_TIMING;
	canCmd.u.openInfo.arbitrationRate	= arbitrationBitRate;
    canCmd.u.openInfo.dataRate			= dataBitRate;
    canCmd.u.openInfo.loopback			= loopbackEnabled;

	bulkOut	= pDevInfo->epAddrs[0].bulkOut;
	bulkIn	= pDevInfo->epAddrs[0].bulkIn;

	pXferBulkOut->buffer			= (uint8_t*) &canCmd;
	pXferBulkOut->length			= canCmd.size;
	pXferBulkOut->actual_length		= canCmd.size;

	// Setup transfer structs for async operation.
	// Bulk in
    pInBuf = malloc( RX_BUF_SIZE );
    libusb_fill_bulk_transfer(
        pXferBulkIn,
        pDevInfo->pDevHandle,
        bulkIn,
        pInBuf,
        RX_BUF_SIZE,
        rxCallBack,
        NULL,
        100);

    pXferBulkIn->flags &= ~(LIBUSB_TRANSFER_FREE_BUFFER | LIBUSB_TRANSFER_FREE_TRANSFER);
    libusb_submit_transfer( pXferBulkIn );

	// Bulk out
    libusb_fill_bulk_transfer( pXferBulkOut, pDevInfo->pDevHandle, bulkOut,
        					  (uint8_t*) NULL, 0, txCallBack, NULL, 1000);

    pXferBulkOut->flags &= ~(LIBUSB_TRANSFER_FREE_BUFFER | LIBUSB_TRANSFER_FREE_TRANSFER);	

	// Send the Open packet.
	canCmd.port							= (uint8_t)portName;
	canCmd.u.openInfo.arbitrationRate	= arbitrationBitRate;
    canCmd.u.openInfo.dataRate			= dataBitRate;
    canCmd.u.openInfo.loopback			= loopbackEnabled;

	pXferBulkOut->buffer			= (uint8_t*) &canCmd;
	pXferBulkOut->length			= canCmd.size;
	pXferBulkOut->actual_length		= canCmd.size;

	bRtn = sendFrame();

	// Check for successful open
	// sendFrame(): returns 0 on sucess, else error code.
	if (!bRtn)
	{
		// Error on sending the port open command.
		if(gAPIDebugLevel >= DEBUG_1)
		{
			printf("\tex_CanOpenPortWithUserTiming: Error opening CAN Port: board %d, port %d, error code %d\n",
				boardName, portName, bRtn);
		}

		cleanUpUSB();

		if(0 == portName)
			gPort0Opened = false;
		else
			gPort1Opened = false;
		
		return(VL_API_CAN_OPEN_PORT_ERROR);
	}
	else
	{
		if(0 == portName)
			gPort0Opened = true;
		else
			gPort1Opened = true;
	}

	// If debug output in MCU is used do this.
    usleep( 1000000 );

	// Send the rxXfer packet to turn on/off sending the transmitted packet
	// back.
	canCmd.port	= portName;
	canCmd.cmd	= USB_CAN_CMD_RX_CTRL;
	canCmd.size	= USB_CAN_CMD_RX_CTRL_SIZE;
	canCmd.misc	= rxEnabled; 
	
	pXferBulkOut->buffer		= (uint8_t*) &canCmd;
	pXferBulkOut->length		= canCmd.size;
	pXferBulkOut->actual_length	= canCmd.size;

	bRtn = sendFrame();

	// Check for success.
	// sendFrame(): returns 0 on sucess, else error code.
	if (!bRtn)
	{
		// Error on sending the port open command.
		if(gAPIDebugLevel >= DEBUG_1)
		{
			printf("\tex_CanOpenPortWithUserTiming: turning off packet receive on transmit: board %d, port %d, error code %d\n",
				boardName, portName, bRtn);
		}

		cleanUpUSB();

		if(0 == portName)
			gPort0Opened = false;
		else
			gPort1Opened = false;
		
		return(VL_API_CAN_OPEN_PORT_ERROR);
	}
	/* ********************************************* */

	/* ***** Done. ***** */
	if (gAPIDebugLevel >= DEBUG_1)
		printf("Leaving ex_CanOpenPortWithUserTiming();\n");
	return(returnCode);
	/* ***************** */
}


/* ****************************************************************************
//  ex_CanClosePort()
//  Closes the handle to a CAN device.
//
//  Inputs:
//		CAN_HANDLE 	*canHandle	=> Handle to the CAN device opened.
//
//	Outputs:
//		None.
//
//  Return values:
//		VSL_API_OK				=> Successfully opened the CAN port.
//		
//  Closes a CAN device. 
   ************************************************************************** */
CmdStatusT ex_CanClosePort(	VL_CANBoardT	boardName, 
							VL_CANPortT		portName)
{
	/* ***** Declarations. ***** */
	CmdStatusT	returnCode;

	bool 			xferError;
	int				i; 					// Simple counter.
	int				rtn;				// libusb return code.
	/* ************************* */
	
	/* ***** Initializations. ***** */
	returnCode	= VL_API_OK;  	// Assume it all works.
	/* **************************** */

	/* ***** Where are we? ***** */
	if (gAPIDebugLevel >= DEBUG_1)
		printf("Entering ex_CanClosePort(): %d;\n", boardName);
	/* ************************** */

	/* ***** Parameter error checking. ***** */
	// Board name.
	if ((boardName < VL_CAN_BOARD0) || (boardName > VL_CAN_BOARD1))
	{
		// Invalid CAN board specified.
		return(VL_API_INVALID_ARG);
	}

	// Port name.
	if ((portName < VL_CAN_PORT0) || (portName > VL_CAN_PORT1))
	{
		// Invalid CAN board specified.
		return(VL_API_INVALID_ARG);
	}
	/* ************************************* */

	/* ***** Make sure USB device is ready to use. ***** */
	// Is the USB device ready to use.
	if (true == gUSBDeviceOpened)
	{
		if (gAPIDebugLevel >= DEBUG_1)
			printf("\tUSB-CAN interface running;\n");
	}
	else
	{
		if (gAPIDebugLevel >= DEBUG_1)
			printf("\tUSB-CAN interface NOT running;\n");

		return(VL_API_CAN_OPEN_DEVICE_ERROR);
	}
	/* ************************************************* */

	/* ***** Send the close port command to the MCU. ***** */
	// Setup the command.
	canCmd.size							= USB_CAN_CMD_CLOSE_SIZE;
	canCmd.cmd							= USB_CAN_CMD_CLOSE;
	canCmd.port							= (uint8_t)portName;

	pXferBulkOut->buffer				= (uint8_t*) &canCmd;
	pXferBulkOut->length				= canCmd.size;
	pXferBulkOut->actual_length			= canCmd.size;

	// Send the Close packet.
	xferError = sendFrame();

	// Do some error checking. 
	// Check if an error was generated by the usb call.
	if (!xferError)
	{
		// USB Error occured.
		returnCode = VL_API_CAN_OPEN_DEVICE_ERROR;
		printf("Board %d: Interface %d, USB error communicating with CAN module.\n",
			boardName, portName);
	}

	gCallBackInfo.txFlag	= 0;

	// Wait for close to be sent.
	gAbortFlag	= false;
	rtn = libusb_handle_events( NULL );
    if ( LIBUSB_SUCCESS != rtn )
    {
        printf( "\nERROR: libusb_handle_events MCAN_CLOSE(1) %s\n", libusb_error_name( rtn ) );
    }

	i = 0;
    while ( !gCallBackInfo.txFlag && !gTxCancelled && !gAbortFlag ) 
	{
		if (i <= 10)
		{
			i++;
		}
		if (i >= 10)
		{
			break;
		}
	}

    i = 0;
    gRxCancelled = false;
    libusb_cancel_transfer( pXferBulkIn );
    while ( !gRxCancelled )
    {
        if ( 10 < i++ )
        {
            break;
        }
        rtn = libusb_handle_events( NULL );
        if ( LIBUSB_SUCCESS != rtn )
        {
            printf( "\nERROR: RX cancel transfer libusb_handle_events %s\n", libusb_error_name( rtn ) );
            break;
        }
    }
	/* *************************************************** */

	/* ***** Leave the USB device connected (opened) for future use. ***** */
	// Close the port though.
	if(0 == portName)
		gPort0Opened = false;
	else
		gPort1Opened = false;
	/* ******************************************************************* */

	/* ***** Done. ***** */
	if (gAPIDebugLevel >= DEBUG_1)
		printf("Leaving ex_CanClosePort(): %d;\n", boardName);

	return(returnCode);
	/* ***************** */
}
/* ***** End of CAN port routines. ***** */


/* ***** CAN Transmit routines. ***** */
/* ****************************************************************************
//  ex_CanTransmit()
//  Transmit the frame.
//	Called after ex_CanOpenPort().
//	This routine assumes that the MCAN_TX_PACKET (standard CAN packet) is
//  filled-out appropriately (i.e. to, t1 and data fields).
//
//  Inputs:
//		VL_CANBoardT 		boardName	=>	Name of the CAN board to open.  This 
//											allows for multiple CAN cards on one 
//											carrier. VL_CAN_BOARD0, VL_CAN_BOARD1.
//		VL_CANPortT 		portName	=>	Name of the CAN port to open. 
//											VL_CAN_PORT0, VL_CAN_PORT1.
//		MCAN_TX_PACKET		*pTxPkt		=>	Pointer to the frame to send out the
//											specifid board and port.
//
//	Outputs:
//		None 
//
//  Return values:
//		VL_API_OK						=> Successfully opened the CAN port
//		VL_API_INVALID_ARG				=> Invalid input passed in.
//		VL_API_CAN_OPEN_DEVICE_ERROR	=> Invalid input passed in.
//		VL_API_CAN_OPEN_NO_DEVICE_FOUND	=> No CAN devices found on carrier board.
//		
   ************************************************************************** */
CmdStatusT ex_CanTransmit(	VL_CANBoardT  boardName, 
							VL_CANPortT   portName,
							MCAN_TX_PACKET *pTxPkt)
{
	/* ***** Declarations. ***** */
	CmdStatusT	returnCode;
	bool			bRtn;
	int 			frameSize	= mcan_length_to_size( (pTxPkt->t1 >> 16) & 0xF );
	int				rtn			= 0;
	/* ************************* */
	
	/* ***** Where are we? ***** */
	if (gAPIDebugLevel >= DEBUG_1)
	{
		printf("Entering ex_CanTransmit();\n");
	}
	
	// More debug if necessary.
	if (gAPIDebugLevel >= DEBUG_2)
	{
		// Print the data field.
        dumpMem(stdout, (uint8_t*) &(pTxPkt->mcanTxData), frameSize);
	}
	/* ************************** */

	/* ***** Parameter error checking. ***** */
	// Board name.
	if ((boardName < VL_CAN_BOARD0) || (boardName > VL_CAN_BOARD1))
	{
		// Invalid CAN board specified.
		return(VL_API_INVALID_ARG);
	}

	// Port name.
	if ((portName < VL_CAN_PORT0) || (portName > VL_CAN_PORT1))
	{
		// Invalid CAN board specified.
		return(VL_API_INVALID_ARG);
	}

	// Is the USB device ready to use.
	if (true == gUSBDeviceOpened)
	{
		if (gAPIDebugLevel >= DEBUG_2)
			printf("\tUSB-CAN interface running;\n");
	}
	else
	{
		if (gAPIDebugLevel >= DEBUG_2)
			printf("\tUSB-CAN interface NOT running;\n");

		return(VL_API_CAN_OPEN_DEVICE_ERROR);
	}

	// Is the port opened?
	if (0 == portName)
	{
		if (false == gPort0Opened)
		{
			if (gAPIDebugLevel >= DEBUG_1)
				printf("\tCAN port %d is closed;\n", portName);

			return(VL_API_CAN_PORT_CLOSED);
		}
	}
	else
	{
		if (false == gPort1Opened)
		{
			if (gAPIDebugLevel >= DEBUG_1)
				printf("\tCAN port %d is closed;\n", portName);

			return(VL_API_CAN_PORT_CLOSED);
		}
	}
	/* ************************************* */

	/* ***** Initializations. ***** */
	returnCode	= VL_API_OK;  // Assume all is OK.
	/* ******************************** */

	/* ***** Setup the command. ***** */
	pTxTest		= &canCmd;
	canCmd.cmd	= (uint8_t)USB_CAN_CMD_TX;
	canCmd.port	= (uint8_t)portName;
	canCmd.misc	= (uint8_t)0;
	canCmd.u.txPacket.t0 = pTxPkt->t0;
	canCmd.u.txPacket.t1 = pTxPkt->t1;

	// Copy the users tranmit packet into the VL CAN transmit packet
	memcpy(&(pTxTest->u.txPacket.mcanTxData), &(pTxPkt->mcanTxData), frameSize);

	// Add the VL packet overhead size to the overall packet size.
	canCmd.size					= USB_CAN_CMD_TX_SIZE + VL_PACKET_OVERHEAD;
	pXferBulkOut->buffer		= (uint8_t*) &canCmd;
	pXferBulkOut->length		= canCmd.size;
	pXferBulkOut->actual_length	= canCmd.size;

	// If requested, print the frame.
	if (gAPIDebugLevel >= DEBUG_1)
	{
		mcan_prt_vl_pkt_header("ex_CanTransmit():", &canCmd);
		if (gAPIDebugLevel >= DEBUG_2)
		{
			mcan_prt_vl(&canCmd);
		}
	}

	// Send the frame.
	bRtn = sendFrame(); // returns 0 on sucess, else error code.
	if (!bRtn)
	{
		// Error on sending the port open command.
		if(gAPIDebugLevel >= DEBUG_1)
		{
			printf("\tex_CanTransmit: Error opening CAN Port: board %d, port %d, error code %d\n",
				boardName, portName, bRtn);
		}

		return(VL_API_CAN_OPEN_PORT_ERROR);
	}
	
	// Setup for the usb events.
	rtn = libusb_handle_events(NULL);
	if (rtn != LIBUSB_SUCCESS)
	{
		if (gAPIDebugLevel >= DEBUG_1)
		{
			printf("libusb_handle_events ERROR: %s;\n", libusb_error_name( rtn ));
		}

		// Return the unsuccessful sending of the packet.
		returnCode = VL_API_CAN_TX_ERROR;
	}
	/* ****************************** */

	/* ***** Done. ***** */
	if (gAPIDebugLevel >= DEBUG_1)
		printf("Leaving VL_CANTransmit();\n");
	return(returnCode);
	/* ***************** */
}


// Place the MCU in update (DFU) mode.
CmdStatusT ex_CanUpdateMCUFwVersion(VL_CANBoardT boardName)
{
	/* ***** Declarations. ***** */
	CmdStatusT returnCode;
	VL_USB_CAN_XFER	canMcuFwVer;
	bool			bRtn;
	//int				tryCount;
	/* ************************* */
	
	/* ***** Initializations. ***** */
	returnCode		= VL_API_OK;
	gMCUFwReceived	= 0;
	//tryCount		= 0;
	/* **************************** */

	if (gAPIDebugLevel >= DEBUG_1)
		printf("Entering ex_CanUpdateMCUFwVersion()\n");

	/* ***** Setup the firmware command (structure). ***** */
	canMcuFwVer.size = USB_CAN_CMD_MCU_PROGRAM_SIZE;
	canMcuFwVer.cmd  = USB_CAN_CMD_MCU_PROGRAM;
	canMcuFwVer.extendedStatus = 0;
	/* *************************************************** */

	/* ***** Send the command. ***** */
	canMcuFwVer.port		 	= 0; // Port does not matter.
	pTxTest					 	= &(canMcuFwVer);
	pXferBulkOut->buffer	 	= (uint8_t *)pTxTest;
	pXferBulkOut->length	 	= pTxTest->size;
    pXferBulkOut->actual_length	= pTxTest->size;

    bRtn = sendFrameRcvFrame( pXferBulkOut );

	if (bRtn)
	{
		// Success.
		if (gAPIDebugLevel >= DEBUG_1)
		{
			printf("MCU now in programming (iap) mode.\n");
		}
	}
	/* ***************************** */

	/* ***** Done. ***** */
	if (gAPIDebugLevel >= DEBUG_1)
		printf("Leaving ex_CanUpdateMCUFwVersion(): %d;\n", returnCode);

	return(returnCode);
	/* ***************** */
}


// Return the MCU firmware version.
CmdStatusT ex_CanGetMCUFwVersion(VL_CANBoardT  boardName,
								 uint32_t *pMcuFwVer)
{
	/* ***** Declarations. ***** */
	CmdStatusT returnCode;
	VL_USB_CAN_XFER	canMcuFwVer;
	bool			bRtn;
	int				tryCount;
	/* ************************* */
	
	/* ***** Initializations. ***** */
	returnCode		= VL_API_OK;
	gMCUFwReceived	= 0;
	tryCount		= 0;
	/* **************************** */

	if (gAPIDebugLevel >= DEBUG_1)
		printf("Entering ex_CanGetMCUFwVersion()\n");

	/* ***** Setup the firmware command (structure). ***** */
	canMcuFwVer.size = USB_CAN_CMD_MCU_FW_VER_SIZE;
	canMcuFwVer.cmd  = USB_CAN_CMD_MCU_FW_VER;
	canMcuFwVer.extendedStatus = 0;
	/* *************************************************** */

	/* ***** Send the command. ***** */
	canMcuFwVer.port		 	= 0; // Port does not matter.
	pTxTest					 	= &(canMcuFwVer);
	pXferBulkOut->buffer	 	= (uint8_t *)pTxTest;
	pXferBulkOut->length	 	= pTxTest->size;
    pXferBulkOut->actual_length	= pTxTest->size;

	// Ask the MCU for it's fw version.
	while (tryCount <= 5)
	{
    	bRtn = sendFrameRcvFrame( pXferBulkOut );

		// The firmware version is set in the Rx callback (rxCallBack()).
		// Not sure what do to with a failed attempt, so leaving it blank for now.
		if (bRtn)
		{
			// Success.
			if (gAPIDebugLevel >= DEBUG_1)
				printf("----->MCU FW Version=0x%06x<-----\n", gMCUFwReceived);
		}

		if (gMCUFwReceived != 0)
		{
			*pMcuFwVer = gMCUFwReceived;
			break;
		}
		else
		{
			tryCount++;
		}
	}
	/* ***************************** */

	/* ***** Done. ***** */
	if (gAPIDebugLevel >= DEBUG_1)
		printf("Leaving ex_CanGetMCUFwVersion(): %d;\n", returnCode);

	return(returnCode);
	/* ***************** */
}


// Return if the specified port is open or closed.
CmdStatusT ex_CanGetPortStatus(VL_CANBoardT boardName, 
	VL_CANPortT portName, VL_CANPortStatusT *pCanPortStatus)
{
	/* ***** Declarations. ***** */
	CmdStatusT returnCode;
	/* ************************* */
	
	/* ***** Initializations. ***** */
	returnCode		= VL_API_OK;
	*pCanPortStatus	= VL_CAN_PORT_UNKNOWN;	// Default to closed.
	/* **************************** */

	/* ***** Where are we ***** */
	if (gAPIDebugLevel >= DEBUG_1)
		printf("Entering ex_CanGetPortStatus()\n");
	/* ************************ */

	/* ***** Parameter error checking. ***** */
	// Board name.
	if ((boardName < VL_CAN_BOARD0) || (boardName > VL_CAN_BOARD1))
	{
		// Invalid CAN board specified.
		return(VL_API_INVALID_ARG);
	}

	// Port name.
	if ((portName < VL_CAN_PORT0) || (portName > VL_CAN_PORT1))
	{
		// Invalid CAN board specified.
		return(VL_API_INVALID_ARG);
	}

	// Is the USB device ready to use.
	if (true == gUSBDeviceOpened)
	{
		if (gAPIDebugLevel >= DEBUG_2)
			printf("\tUSB-CAN interface running;\n");
	}
	else
	{
		if (gAPIDebugLevel >= DEBUG_2)
			printf("\tUSB-CAN interface NOT running;\n");

		return(VL_API_CAN_OPEN_DEVICE_ERROR);
	}

	// Is the port opened?
	if (0 == portName)
	{
		if (false == gPort0Opened)
		{
			if (gAPIDebugLevel >= DEBUG_1)
				printf("\tCAN port %d is closed;\n", portName);

			*pCanPortStatus	= VL_CAN_PORT_CLOSED;
			return(VL_API_CAN_PORT_CLOSED);
		}
		else
		{
			if (gAPIDebugLevel >= DEBUG_1)
				printf("\tCAN port %d is opened;\n", portName);

			*pCanPortStatus	= VL_CAN_PORT_OPENED;
			return(VL_API_OK);
		}
	}
	else
	{
		if (false == gPort1Opened)
		{
			if (gAPIDebugLevel >= DEBUG_1)
				printf("\tCAN port %d is closed;\n", portName);

			*pCanPortStatus	= VL_CAN_PORT_CLOSED;
			return(VL_API_CAN_PORT_CLOSED);
		}
		else
		{
			if (gAPIDebugLevel >= DEBUG_1)
				printf("\tCAN port %d is opened;\n", portName);

			*pCanPortStatus	= VL_CAN_PORT_OPENED;	// Default to closed.
			return(VL_API_OK);
		}
	}
	/* ************************************* */

	return(returnCode);
}

