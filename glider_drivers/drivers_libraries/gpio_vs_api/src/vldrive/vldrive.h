/*
 * vldrive.h
 *
 *  Created on: Aug 19, 2013
 *      Author: Andy Dowie
 */

#ifndef VLDRIVE_IOCTL_H
#define VLDRIVE_IOCTL_H
#include <linux/ioctl.h>

typedef struct
{
	unsigned char	Address;
	union {
	unsigned char  	Data8;
	unsigned short 	Data16;
	unsigned long  	Data32;
	}Passed;
} VLDriveArg;


#define IRQ_CONTROL_OFF		0x03
#define IRQ_STATUS_OFF		0x04
#define SPI_CONTROL_OFF		0x08
#define SPI_STATUS_OFF		0x09
#define SPI_DATA0_OFF		0x0A
#define SPI_DATA1_OFF		0x0B
#define SPI_DATA2_OFF		0x0C
#define SPI_DATA3_OFF		0x0D
#define SPI_CUSTOM_OFF		0x0E
#define SPI_ADCONSTAT_OFF	0x0F
#define IRQ_DIO_STATUS_OFF  0x26
#define TMR_TCNTRL			0x05
#define TMR_TBASEMS			0x06
#define TMR_TBASELS			0x07
#define SIG_VL_TIMER		56
#define SIG_VL_DIO		    57


#define	IOCTL_VSL_READ_PORT_UCHAR	_IOWR(0xAA, 1, VLDriveArg *)
#define	IOCTL_VSL_READ_PORT_USHORT  _IOWR(0xAA, 2, VLDriveArg *)
#define	IOCTL_VSL_READ_PORT_ULONG   _IOWR(0xAA, 3, VLDriveArg *)

#define	IOCTL_VSL_WRITE_PORT_UCHAR	_IOWR(0xAA, 4, VLDriveArg *)
#define	IOCTL_VSL_WRITE_PORT_USHORT _IOWR(0xAA, 5, VLDriveArg *)
#define	IOCTL_VSL_WRITE_PORT_ULONG  _IOWR(0xAA, 6, VLDriveArg *)

#define	VSL_IOCTL_PASS_TIMER_PID    _IOWR(0xAA, 7, pid_t *)
#define	VSL_IOCTL_PASS_DIO_PID      _IOWR(0xAA, 8, pid_t *)
#define	VSL_IOCTL_GET_TMR_IRQ      	_IOWR(0xAA, 9, long *)
#define	VSL_IOCTL_GET_DIO_IRQ      	_IOWR(0xAA, 10, long *)


#define VLDRIVE_MINOR	0xAE
#define VLDRIVE_CNT		1
#define FPGA_BASE_DEF 0xC80
#define FPGA_IRQ_DEF 5



#endif

