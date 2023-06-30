/******************************************************************************/
/* Name			: VL_CAN.h												      */
/*                                                                            */
/* Contents		: Function prototypes for Operating System Agnostic calls     */
/*                                                                            */
/* Author		: VersaLogic Corporation					                  */
/*                                                                            */
/* Version		: 1.0.0                                                       */
/*                                                                            */
/* Copyright 2018 VersaLogic Corporation									  */
/******************************************************************************/


#ifndef _VLAPICAN_H
#define _VLAPICAN_H

/* ***** Includes ***** */
#include <sys/types.h>
#include <stdint.h>
#include <libusb-1.0/libusb.h>
//#include <stdbool.h>
/* ******************** */

/* ***** From nxp-can.h ***** */


#ifndef __NXP_CAN_H__
#define __NXP_CAN_H__

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: There maybe endian issues with anything other than uint8_t items
// depending on host endian. The default for the LPC54616 is little endian and
// the Atom is little endian as far as I can tell. We'll find out.
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// These defines and structures are derived from the LPC54616 user manual V2.2.

// 35.9 RX buffer and FIFO element, pg 729
// 35.10 Tx buffer element, pg 731

// R0 and T0
#define CAN_RX_TX_ID_MASK           (0x1FFFFFFF)
#define CAN_RX_TX_ID11_SHFT         (18)
#define CAN_RX_TX_REMOTE_FRAME      (0x20000000)
#define CAN_RX_TX_EXTENDED_ID       (0x40000000)
#define CAN_RX_TX_ERR_PASSIVE       (0x80000000)

#define CAN_RX_TX_GET_ID( var )             ((var & CAN_RX_TX_ID_MASK) >> CAN_RX_TX_ID11_SHFT)
#define CAN_RX_TX_SET_ID( var, id )         ((var & ~CAN_RX_TX_ID_MASK) | (id << CAN_RX_TX_ID11_SHFT))
#define CAN_RX_TX_GET_EXID( var )           (var & CAN_RX_TX_ID_MASK)
#define CAN_RX_TX_SET_EXID( var, id )       ((var & ~CAN_RX_TX_ID_MASK) | id)

// R1 and T1
#define CAN_RX_TX_SIZE_0            (0x00000000)
#define CAN_RX_TX_SIZE_1            (0x00010000)
#define CAN_RX_TX_SIZE_2            (0x00020000)
#define CAN_RX_TX_SIZE_3            (0x00030000)
#define CAN_RX_TX_SIZE_4            (0x00040000)
#define CAN_RX_TX_SIZE_5            (0x00050000)
#define CAN_RX_TX_SIZE_6            (0x00060000)
#define CAN_RX_TX_SIZE_7            (0x00070000)
#define CAN_RX_TX_SIZE_8            (0x00080000)
#define CAN_RX_TX_SIZE_12           (0x00090000)
#define CAN_RX_TX_SIZE_16           (0x000A0000)
#define CAN_RX_TX_SIZE_20           (0x000B0000)
#define CAN_RX_TX_SIZE_24           (0x000C0000)
#define CAN_RX_TX_SIZE_32           (0x000D0000)
#define CAN_RX_TX_SIZE_48           (0x000E0000)
#define CAN_RX_TX_SIZE_64           (0x000F0000)
#define CAN_RX_TX_BR_SWITCHING      (0x00100000)
#define CAN_RX_TX_CAN_FD_FORMAT     (0x00200000)
#define CAN_RX_TX_CAN_TX_EVENT      (0x00800000)

#define CAN_RX_TX_SET_SIZE( var, size )     ((var & 0xFFF0FFFF) | size)
#define CAN_RX_TX_GET_SIZE( var )           (var & 0x000F0000)

// R1
#define CAN_RX_TIMESTAMP_MASK       (0x0000FFFF)
#define CAN_RX_TIMESTAMP_SHFT       (0)
#define CAN_RX_FILTER_INDEX_MASK    (0x7E000000)
#define CAN_RX_FILTER_INDEX_SHFT    (24)
#define CAN_RX_FILTER_NO_MATCH      (0x80000000)

// T1
#define CAN_TX_USE_TX_EVENTS        (0x00800000)
#define CAN_TX_MSG_MARKER_MASK      (0xFF000000)
#define CAN_TX_MSG_MARKER_SHFT      (24)

#define CAN_MAX_FRAME_SIZE          (64)

// Status values returned in misc when status is called or returned.
#define CAN_STS_OPENED              (0x80)      // Upper bit set for open
#define CAN_STS_OK                  (0)
#define CAN_STS_ERROR               (1)
#define CAN_STS_TX_OK               (2)         // Send when tx FIFO is ready
#define CAN_STS_TX_BUSY             (3)         // Send when tx FIFO hits high
#define CAN_STS_TX_EVENT            (4)         // Send when tx FIFO hits high
#define CAN_STS_TX_ERROR            (5)
#define CAN_STS_RATE_ERROR          (6)
#define CAN_STS_INVALID_PORT        (7)
#define CAN_STS_BUS_ERROR           (8)
#define CAN_STS_UNDEFINED           (0x7F)

// MCU firmware version information
#define CAN_MCU_FW_VER_MAJOR			(0U)
#define CAN_MCU_FW_VER_MINOR			(1U)
#define CAN_MCU_FW_VER_RELEASE			(5U)
#define VL_MCU_MAKE_VERSION(major, minor, release) (((major) << 16) | ((minor) << 8) | (release))

/* Command return codes. */
typedef enum {
	VL_API_OK			= 1,			// Function completed successfully.
	VL_API_ERROR,						// Unexpected problem occurred in the VersaLogic library.
	VL_API_INVALID_ARG,					// Command arguemnt out of range.
	VL_API_NOT_SUPPORTED,				// Function not supported.
	VL_API_ARG_NOT_SUPPORTED,			// Arguement currently not supported.
	VL_API_CAN_CLOSE_DEVICE_ERROR,		// Cound not close the CAN device.
	VL_API_CAN_OPEN_DEVICE_ERROR, 		// Cound not open the CAN device.
	VL_API_CAN_OPEN_NO_DEVICE_FOUND, 	// Cound not find any USB (CAN) devices.
	VL_API_CAN_OPEN_NO_DEVICE_DESC,	 	// Cound not open the CAN device descriptor.
	VL_API_CAN_OPEN_PORT_ERROR,		 	// Cound not open the CAN device descriptor.
	VL_API_CAN_PORT_CLOSED,				// CAN port is closed.
	VL_API_CAN_TX_ERROR		 			// Failed to transmit CAN packet.
} CmdStatusT;


//
// MCAN_TX_PACKET is an image of the MCAN TX buffer element in message area.
//
typedef struct _mcan_tx_packet_ {
    uint32_t            t0;
    uint32_t            t1;
    uint8_t             mcanTxData[ CAN_MAX_FRAME_SIZE ];
} MCAN_TX_PACKET;

//
// MCAN_TX_DONE is an image of the MCAN TX event E0 and E1 data.
//
typedef struct _mcan_tx_done_ {
    uint32_t            e0;
    uint32_t            e1;
} MCAN_TX_DONE;

//
// MCAN_RX_PACKET is an image of the MCAN RX buffer element in message area.
//
typedef struct _mcan_rx_packet_ {
    uint32_t            r0;
    uint32_t            r1;
    uint8_t             mcanRxData[ CAN_MAX_FRAME_SIZE ];
} MCAN_RX_PACKET;

//
// Contains items used on open. Bit rates and loop back can only be done on open.
//
typedef struct _mcan_open_ {
    uint32_t  arbitrationRate;   // In BPS
    uint32_t  dataRate;          // In BPS
    uint8_t   loopback;          // 'true' internal loop back
    uint8_t   rxEnabled;         // 
    uint32_t  masterDivider;     //
    uint16_t  arbPreDivider;     // Arbitration Clock Pre-scaler Division Factor.
    uint8_t   arbRJumpwidth;     // Arbitration Re-sync Jump Width.
    uint8_t   arbSeg1;           // Arbitration Data Time Segment 1.
    uint8_t   arbSeg2;           // Arbitration Data Time Segment 2.
    uint16_t  dataPreDivider;    // Data Clock Pre-scaler Division Factor.
    uint8_t   dataRJumpwidth;    // Data Re-sync Jump Width.
    uint8_t   dataSeg1;          // Data Data Time Segment 1.
    uint8_t   dataSeg2;          // Data Data Time Segment 2.
} MCAN_OPEN;

//
// Contains MCU fW version information.
//
typedef struct _mcan_mcu_fw_ver_ {
    uint8_t            major;        // Hex values
    uint8_t            minor;
    uint8_t            release;
} MCAN_MCU_FW_VER;

//
// This is the packet send across the USB interface to control CAN and
// do CAN IO.
//
#define USB_CAN_CMD_UNDEFINED       (0)
#define USB_CAN_CMD_OPEN            (1)
#define USB_CAN_CMD_CLOSE           (2)
#define USB_CAN_CMD_TX              (3)
#define USB_CAN_CMD_RX              (4)
#define USB_CAN_CMD_STS             (5)
#define USB_CAN_CMD_ID              (6)     // Blink board LED at a fast rate
#define USB_CAN_CMD_MCU_PROGRAM     (7)
#define USB_CAN_CMD_RX_CTRL         (8)     // Used to turn MCAN RX on/off with misc
#define USB_CAN_CMD_MCU_FW_VER		(9)		// Return the version of the MCU code, major, minor, release
#define USB_CAN_CMD_MCU_REG_VAL     (10)	// Return the version of the MCU code, major, minor, release
#define USB_CAN_CMD_OPEN_USR_TIMING (11)	// Return the version of the MCU code, major, minor, release
#define USB_CAN_CMD_MAX             (12)     // Number of commands, add command, CHANGE!

#define USB_CAN_CMD_HDR_SIZE        (4)
#define USB_CAN_CMD_UNDEF_SIZE      (USB_CAN_CMD_HDR_SIZE)
#define USB_CAN_CMD_OPEN_SIZE       (USB_CAN_CMD_HDR_SIZE + sizeof( MCAN_OPEN ))
#define USB_CAN_CMD_CLOSE_SIZE      (USB_CAN_CMD_HDR_SIZE)
#define USB_CAN_CMD_TX_SIZE         (USB_CAN_CMD_HDR_SIZE + sizeof( MCAN_TX_PACKET ))
#define USB_CAN_CMD_TX_DONE_SIZE    (USB_CAN_CMD_HDR_SIZE + sizeof( MCAN_TX_DONE ))
#define USB_CAN_CMD_RX_SIZE         (USB_CAN_CMD_HDR_SIZE + sizeof( MCAN_RX_PACKET ))
#define USB_CAN_CMD_STS_SIZE        (USB_CAN_CMD_HDR_SIZE)      // Status returned in misc.
#define USB_CAN_CMD_ID_SIZE         (USB_CAN_CMD_HDR_SIZE)
#define USB_CAN_CMD_RX_CTRL_SIZE    (USB_CAN_CMD_HDR_SIZE)
#define USB_CAN_CMD_OVERHEAD_SIZE   (12)        // Offset into VL_USB_CAN_XFER for rx/tx data
#define USB_CAN_CMD_MCU_FW_VER_SIZE (USB_CAN_CMD_HDR_SIZE) + sizeof( MCAN_MCU_FW_VER )   // Version returned in misc.
#define USB_CAN_CMD_MCU_PROGRAM_SIZE (USB_CAN_CMD_HDR_SIZE)
#define USB_CAN_CMD_OPEN_USR_TIMING_SIZE (uint8_t)(USB_CAN_CMD_HDR_SIZE + sizeof( MCAN_OPEN ) + 4)

typedef struct _usb_can_packet_ {
    uint8_t             size;
    uint8_t             cmd;
    uint8_t             port;
    uint8_t             misc;
    uint32_t			extendedStatus;
    union {
        MCAN_OPEN           openInfo;
        MCAN_RX_PACKET      rxPacket;
        MCAN_TX_PACKET      txPacket;
        MCAN_TX_DONE        txEvent;
    } u;
} VL_USB_CAN_XFER;


// User Defined function called when a packet is recieved.
void userDefinedRxFunction(VL_USB_CAN_XFER *pRxPacket, int status);

#endif // __NXP_CAN_H__

/* ************************** */


/* ***** Defines. ***** */
#define CAN_STR_LEN 64						// Generic string length.
#define TRUE 1								// Simple define.
#define true 1								// Simple define.
#define FALSE 0								// Simple define.
#define false 0								// Simple define.
#define ON 1
#define OFF 0
#define VL_ENABLE_RX_ON_TX		(1U)
#define VL_DISABLE_RX_ON_TX 	(0U)
#define VL_ENABLE_LOOPBACK		(1U)
#define VL_DISABLE_LOOPBACK		(0U)

/* USB defines. */
#define USB_CLASS_CDC_MODEM             ( 2 )       // CDC/Comm Modem
#define USB_CLASS_CDC_DATA              ( 10 )      // CDC/Comm Data

#define MAX_NUM_USB_DEVICES 2		// Maximum number of USB devies per board.
#define NUM_USBS_PER_DEVICE ( 2 )   // Number of USB channels per device.

// lsusb -v -d 0x1fc9:  <= lsusb command to get our device info.
#define OUR_VENDOR          ( 0x1FC9 )  // NXP vendor ID

// NXP Composit 2 VCOM product code.
#define OUR_PRODUCT         ( 0x00A3 )

// NXP VCOM Example enumeration
#define EP_BULK_IN          ( 0x83 )
#define EP_BULK_OUT         ( 0x03 )

// Debug enabling code 
#define	DEBUG_OFF	0	// No debug messages printed.
#define DEBUG_1		1	// Some debug messages printed.
#define DEBUG_2		2 	// More debug messages printed.

/* ******************** */

/* ***** Types/Structures. ***** */
typedef unsigned int uint32_t;
typedef int bool;

typedef struct
{
	unsigned char szVersionString[CAN_STR_LEN];
	unsigned char szDevicePath[CAN_STR_LEN];
	int			  deviceDesc;
} CAN_HANDLE, *pCAN_HANDLE;

// Storage for endpoints used by interfaces.
struct xfaceEpAddrs {
    int                                 canPort;        // 0 or 1
    uint8_t                             control;        // Modem ep address
    uint8_t                             bulkIn;         // Data input ep address
    uint8_t                             bulkOut;        // Data output ep address
};

struct deviceAccessInfo {
    struct libusb_device_descriptor     devDescriptor;
    libusb_device                       *pLibUsbDev;
    libusb_device_handle                *pDevHandle;
    int                                 numInterfaces;
    struct xfaceEpAddrs                 epAddrs[ NUM_USBS_PER_DEVICE ];
};

struct deviceAccessInfo devicesInfoEpAddrs[MAX_NUM_USB_DEVICES];
libusb_context          *pUsbContext; 

// Define the names for two CAN boards.
typedef enum
{
	VL_CAN_BOARD0	= 0,
	VL_CAN_BOARD1	
} VL_CANBoardT;

// Define the names for two CAN ports (per board).
typedef enum
{
	VL_CAN_PORT0	= 0,
	VL_CAN_PORT1	
} VL_CANPortT;

// Define the Port status.
typedef enum
{
	VL_CAN_PORT_CLOSED	= 0,
	VL_CAN_PORT_OPENED,
	VL_CAN_PORT_UNKNOWN
} VL_CANPortStatusT;

// Define MPEe-C1E card status.
typedef enum
{
	VL_CAN_BOARD_CLOSED = 0,
	VL_CAN_BOARD_OPENED
} VL_CANBoardStatusT;
/* ***************************** */


/* ***** Globals ***** */
extern int gAPIDebugLevel;		// Setting verbosity of commands.
extern bool gAbortFlag;
/* ******************* */


/* CAN configuration.
	nominalBaudRate		=> Nominal bit rate in bits/second. For
						   classical CAN and CAN-FD frames that do not
						   use bit rate switching this is the speed the
						   frames will be tranmitted at.
	dataBaudRate		=> Bit rate for the data byte and checksum in
						   bit/second. Only used for CAN-FD frames that
						   use bit rate swtiching. Set to zero to
						   disable bit rate switching. Must be higher
								   than nominalBaudRate.
	baseAddress			=> Starting address in SRAM for data structures
						   used by the CAN controller, such as message
						   buffers, FIFOs and filters.
	timestampClock_Hz	=> The frequency of the timestamp clock in Hz.
						   Used for timestamping received messages.
						   Zero disables timestamping.
	rejectStandardRTR	=> If TRUE, then all 11-bit RTR frames received
						   will be ignored.
	rejectExtendedRTR	=> If TRUE, then all 29-bit RTR frames received 
						   will be ignored.
	enableLoopBack		=> Set to TRU to enable loopback mode for self-test.
	enableNonISOMode	=> Set to TRUE to use Bosch CAN-FD frame format (not
						   recommended). Set to FALSE to use ISO CAN-FD frame
						   format.
	disableFD			=> Set to TRUE to disable CAN-FD. All transmitted
						   frames will be classical frames. Any CAN-FD frames
						   will be converted into error frames.
*/
typedef struct
{
	uint32_t	nominalBaudRate;
	uint32_t	dataBaudRate;
	uint32_t	baseAddress;
	uint32_t	timestampClock_Hz;
	bool		rejectStandardRTR;
	bool		rejectExtendedRTR;
	bool		enableLoopBack;
	bool		enableNonISOMode;
	bool		disableFD;
} CAN_CONFIG, *pCAN_CONFIG;
/* *********************** */

/* ex_CanOpenPort()
 * Opens the specified CAN port with the specified configuration. 
 *
 *  Return values:
 *		VL_API_OK						=> Successfully opened the CAN port
 *		VL_API_INVALID_ARG				=> Invalid input passed in.
 *		VL_API_CAN_OPEN_DEVICE_ERROR	=> Invalid input passed in.
 *		VL_API_CAN_OPEN_NO_DEVICE_FOUND	=> No CAN devices found on carrier board.
 */
CmdStatusT ex_CanOpenPort(VL_CANBoardT	boardName, 
											 VL_CANPortT	portName, 
											 uint32_t		arbitrationBitRate,
											 uint32_t		dataBitRate,
											 uint32_t		rxEnabled,
											 uint32_t  		loopbackEnabled);


/* VSL_CanSetPortTimingParameters()
 * Opens the specified CAN port with the specified configuration. 
 * Let's the user set all CAN Port timing parameters.
 *
 * Return values:
 *     None
 */
CmdStatusT ex_CanOpenPortWithUserTiming( VL_CANBoardT boardName, 
										  VL_CANPortT  portName,
                                          uint32_t	   arbitrationBitRate,
                                          uint32_t	   dataBitRate,
										  uint8_t      rxEnabled,
										  uint8_t      loopbackEnabled,
                                          uint32_t	   masterDivider,
                                          uint16_t     arbPreDivider,
                                          uint8_t      arbResyncJumpWidth,
                                          uint8_t      arbSeg1,
                                          uint8_t      arbSeg2,
                                          uint16_t     dataPreDivider,
                                          uint8_t      dataResyncJumpWidth,
                                          uint8_t      dataSeg1,
                                          uint8_t      dataSeg2);


/*  ex_CanClosePort()
 *  Closes the pecified CAN port.
 *
 *  Return values:
 *		VSL_API_OK				=> Successfully opened the CAN port.
 */
CmdStatusT ex_CanClosePort(	VL_CANBoardT	boardName, 
												VL_CANPortT		portName);

/*  ex_CanTransmit()
 *  Transmit the frame.
 *	Called after ex_CanOpenPort().
 *	This routine assumes that the MCAN_TX_PACKET (standard CAN packet) is
 *  filled-out appropriately (i.e. t0, t1 and data fields).
 *
 *  Return values:
 *		VL_API_OK						=> Successfully opened the CAN port
 *		VL_API_INVALID_ARG				=> Invalid input passed in.
 *		VL_API_CAN_OPEN_DEVICE_ERROR	=> Invalid input passed in.
 *		VL_API_CAN_OPEN_NO_DEVICE_FOUND	=> No CAN devices found on carrier board.
 */
CmdStatusT ex_CanTransmit(	VL_CANBoardT  boardName, 
												VL_CANPortT   portName,
												MCAN_TX_PACKET *pTxPkt);


CmdStatusT ex_CanGetMCUFwVersion(VL_CANBoardT  boardName,
	uint32_t *pMcuFwVer);

#endif  // _VLAPI_H_

