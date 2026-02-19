/***************************************************************************
                          tmlcomm.h  -  description
                             -------------------
    begin                : Mon Mar 17 2003
    copyright            : (C) 2003 by Alin Nastac
    email                : a_nastac@technosoft.ch
***************************************************************************/

#ifndef _TMLCOMM_H
#define _TMLCOMM_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

/* Type defs used in the following structs */
#ifndef BYTE
#define BYTE u_int8_t
#endif
#ifndef WORD
#define WORD u_int16_t
#endif
#ifndef DWORD
#define DWORD u_int32_t
#endif
#ifndef ASSERT
#define ASSERT(x) assert(x)
#endif

/* used in MSK_CallbackDebugLog function */
#define LOG_ERROR      1
#define LOG_WARNING   2
#define LOG_TRAFFIC   3

//supported CAN protocols
#define PROTOCOL_TMLCAN					0x00 //use TMLCAN protocol (default, 29-bit identifiers)
#define PROTOCOL_TECHNOCAN				0x80 //use TechnoCAN protocol (11-bit identifiers)
#define PROTOCOL_MASK					0x80 //this bits are used for specifying CAN protocol through nChannelType param of MSK_OpenComm function

/***** supported CAN devices *****************************
CHANNEL_IXXAT_CAN - see http://www.ixxat.com
CHANNEL_SYS_TEC_USBCAN - see www.systec-electronic.com
CHANNEL_ESD_CAN - see http://www.esd-electronics.com
CHANNEL_PEAK_SYS_PCAN_* - see http://www.peak-system.com
CHANNEL_LAWICEL_USBCAN - see http://www.canusb.com
**********************************************************/

#define CHANNEL_RS232                0
#define CHANNEL_RS485                1
#define CHANNEL_IXXAT_CAN            2
#define CHANNEL_SYS_TEC_USBCAN       3 /*Not supported on this OS */
#define CHANNEL_PEAK_SYS_PCAN_PCI    4
#define CHANNEL_ESD_CAN              5
#define CHANNEL_PEAK_SYS_PCAN_ISA    6
#define CHANNEL_PEAK_SYS_PCAN_PC104  CHANNEL_PEAK_SYS_PCAN_ISA /*Same with PCAN_ISA*/
#define CHANNEL_PEAK_SYS_PCAN_USB    7
#define CHANNEL_PEAK_SYS_PCAN_DONGLE 8
#define CHANNEL_LAWICEL_USBCAN       9
#define CHANNEL_VIRTUAL_SERIAL       15 /*Not supported on this OS */
#define CHANNEL_XPORT_IP             16

#define IS_REAL_SERIAL_TYPE(type) ((type) <= CHANNEL_RS485)
#define IS_SERIAL_TYPE(type) IS_REAL_SERIAL_TYPE(type)
#define IS_CAN_TYPE(type) ((int)(type) >= CHANNEL_IXXAT_CAN && (int)(type) <= CHANNEL_LAWICEL_USBCAN)
#define IS_IP_TYPE(type) ((int)(type) == CHANNEL_XPORT_IP)

#define DSP_PC_FLAG         0x1
#define DSP_GROUP_FLAG      0x1000

/*****************************************************************************
MSK Message structure
*****************************************************************************/
#pragma pack(push, 1)
typedef struct
{
   WORD m_Len;
   WORD m_Addr;
   WORD m_OpCode;
   WORD m_Data[4];
} MSK_MSG;
#pragma pack(pop)

typedef const char* PCSTRING;
#ifndef __cplusplus
	typedef int bool;
#	define false 0
#	define true 1
#endif

/****************************************************************************
Callback function used by client application for handling debug messages
****************************************************************************/
typedef void (*MSK_CallbackDebugLog)(PCSTRING pszMsg);

/****************************************************************************
Callback function used by client application for handling unsolicited
messages which this driver receives in unexpected places
****************************************************************************/
typedef void (*MSK_CallbackRecvMsg)(MSK_MSG* pMsg);

/****************************************************************************
 Callback function used by client application to manage complex
 MSK functions, like downloading
****************************************************************************/
typedef bool (*MSK_Callback)(int nMSGType, PCSTRING pcszMsg,
                             float nElpsTime, void * pUsrData);
#define MSK_CALLBACK_APPEND_MSG				1
#define MSK_CALLBACK_YESNO_MSG				2
#define MSG_CALLBACK_REPLACE_LASTLINE		3
#define MSG_CALLBACK_APPEND_NEWLINE_MSG	4
/*	Parameters:
	nMSGType:	message type;
	pcszMsg:	message to be displayed by client application;
				the meaning of this message depends by the "nMSGType" parameter:
					"MSK_CALLBACK_APPEND_MSG" - append the message to the
						client application output;
					"MSK_CALLBACK_YESNO_MSG" - prompts user for a question;
					"MSG_CALLBACK_REPLACE_LASTLINE" - replace last line of the
						client application output with the provided message;
					"MSG_CALLBACK_APPEND_NEWLINE_MSG" - append the message at
						the beginning of a new line;
	nElpsTime:	relative elapsed time (percent value);
	pUsrData:	pointer to user data provided by client application;
				usually used to identify the output device.
	Return value:
				TRUE:	continue to execute MSK function;
				FALSE:	exit from MSK function with user abort error code; */

/*	Macros used to identify the DSP memory space */
#define MSK_PM		0	/*Program memory space */
#define MSK_DM		1	/*Data memory space */
#define MSK_SPI	2	/*SPI memory space */

#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************/
/* All MSK exported function use the same meaning of the return value:
	TRUE:	if the function is successful;
	FALSE:	error; Use MSK_GetLastError for more details*/
/*****************************************************************************/
PCSTRING MSK_GetLastError(void);

/*****************************************************************************
   MSK_RegisterDebugLogHandler: set destination of debugging messages and level of debugging
*****************************************************************************/
void MSK_RegisterDebugLogHandler(int nLevel, MSK_CallbackDebugLog pfnCallback);
/*****************************************************************************
   Parameters:
   pfnCallback - pointer to callback function who receive debug messages
   nLevel - level of logging (1 = error, 2 = error and warning, 3 = traffic, error and warning)
   if pfnCallback == NULL or nLevel <=0, logging is disabled
*****************************************************************************/

/****************************************************************************
MSK_RegisterReceiveUnsolicitedMsgHandler:
	register a callback called when unexpected messages are received instead acknowledge
*****************************************************************************/
void MSK_RegisterReceiveUnsolicitedMsgHandler(MSK_CallbackRecvMsg pfnCallback);
/*
	pfnCallback: pointer to the callback function
*/

/*****************************************************************************
   MSK_OpenComm: function that opens a communication channel
*****************************************************************************/
#ifdef __cplusplus
int MSK_OpenComm(PCSTRING pszComm, WORD nHostAddress, bool bExclusive, BYTE nChannelType = CHANNEL_RS232);
#else
int MSK_OpenComm(PCSTRING pszComm, WORD nHostAddress, bool bExclusive, BYTE nChannelType);
#endif
/*****************************************************************************
   Parameters:
      pszComm:   communication device inode
            CHANNEL_RS232, CHANNEL_RS485: "/dev/ttyS0" (equivalent for COM1), "/dev/ttyS1", ...
            CHANNEL_IXXAT_CAN : "/dev/can0:0" ("device file:CAN controller number" ; default CAN controller number is 0)
            CHANNEL_PEAK_SYS_* : "/dev/pcan0", "/dev/pcan1", ...
            CHANNEL_ESD_CAN : "/dev/can0" or "0", "/dev/can1" or "1", ...
				CHANNEL_LAWICEL_USBCAN: "/dev/ttyUSB0", ...
				CHANNEL_XPORT_IP: "IP" or "hostname"
      nHostAddress: host board address
      nChannelType: Type of communcation, specified as (PROTOCOL_* | CHANNEL_*). 
				If not specified, protocol is PROTOCOL_TMLCAN.
		bExclusive:	TRUE value for exclusive using.
   Return:
      -1 means error
      otherwise is the port file descriptor.
*****************************************************************************/

/*****************************************************************************
   MSK_SelectComm: function that select a communication channel.
      Note: MSK_OpenComm automatically selects the communication device. If you work only
         with one communication device there is no need to call this function
*****************************************************************************/
int MSK_SelectComm(int fdComm);
/*****************************************************************************
   Parameters:
      fdComm: the communication device file descriptor.
   Return:
      if error it returns -1
		else returns previous selected fdComm
*****************************************************************************/

/*****************************************************************************
   MSK_CloseComm: function that closes the communication channel 
*****************************************************************************/
#ifdef __cplusplus
bool MSK_CloseComm(int fdComm = -1);
#else
bool MSK_CloseComm(int fdComm);
#endif
/*****************************************************************************
   Parameters:
      fdComm: the communication device file descriptor. -1 means current selected.
*****************************************************************************/

/*****************************************************************************
	MSK_UpdateSettings: function that reloads settings from /etc/tmlcomm.conf
*****************************************************************************/
bool MSK_UpdateSettings(void);

/*****************************************************************************
   MSK_SetBaudRate: set device baud rate.
*****************************************************************************/
bool MSK_SetBaudRate(DWORD nBaud);
/*****************************************************************************
	Parameters:
		nBaud:		new baud rate
	       Serial lines have baud rates of 9600, 19200, 38400, 57600 or 115200.
			 CAN interfaces have baud rates of 125000, 250000, 500000, 800000 
			 (not supported by ESD drivers) or 1000000. 
*****************************************************************************/

/*****************************************************************************
   MSK_GetBaudRate: get serial baud rate.
*****************************************************************************/
DWORD MSK_GetBaudRate(void);
/*****************************************************************************
   Return: 0 on error or baudrate on success
*****************************************************************************/

/*****************************************************************************
	MSK_GetBytesCountInQueue: Test if any character is available on the communication buffer device
*****************************************************************************/
int MSK_GetBytesCountInQueue(void);
/*****************************************************************************
	Returns:
	if < 0		erorr
	else number of bytes in serial queue
*****************************************************************************/

/*****************************************************************************
   MSK_SetActiveDrive: set active drive (messages are sent/received to/from this axis)
*****************************************************************************/
bool MSK_SetActiveDrive(WORD AxisID);
/*****************************************************************************
   Parameters:
      AxisID: the address of the drive.
*****************************************************************************/

/*****************************************************************************
   MSK_GetActiveDrive: retrieve the current active drive
*****************************************************************************/
bool MSK_GetActiveDrive(WORD* pAxisID);
/*****************************************************************************
   Parameters:
      pAxisID: pointer to a buffer which will contain the address of the drive.
*****************************************************************************/

/*****************************************************************************
   MSK_SetHostDrive: Set the address of the host drive
*****************************************************************************/
bool MSK_SetHostDrive(WORD AxisID);
/*****************************************************************************
   Parameters:
      AxisID: the address of the drive.
*****************************************************************************/

/*****************************************************************************
   MSK_GetHostDrive: Get the address of the host drive
*****************************************************************************/
bool MSK_GetHostDrive(WORD* pAxisID);
/*****************************************************************************
   Parameters:
      pAxisID: pointer to a buffer which will contain the address of the drive.
*****************************************************************************/

/*****************************************************************************
   MSK_GetDriveVersion: retrieve firmware version of the active drive
*****************************************************************************/
bool MSK_GetDriveVersion(char* szDriveVersion);
/*****************************************************************************
   Parameters:
      szDriveVersion: pointer to a buffer of 5 characters (not including null terminator)
         which will contain the firmware version.
*****************************************************************************/

/*****************************************************************************
   MSK_ResetDrive: reset the active drive.
*****************************************************************************/
bool MSK_ResetDrive(void);

/*****************************************************************************
   MSK_SendMessage: send a message.
		Note: The active drive has no effect on this function.
*****************************************************************************/
bool MSK_SendMessage(MSK_MSG *pMsg);
/*****************************************************************************
   Parameters:
      pMsg: pointer to a message structure which will be sent
   CAUTION: pMsg->wAddr must be equal with (axisID << 4) or ((1 << (4+groupID-1)) | DSP_GROUP_FLAG)
*****************************************************************************/

/*****************************************************************************
   MSK_ReceiveMessage: receive a message. The active drive has no effect on this function.
*****************************************************************************/
bool MSK_ReceiveMessage(MSK_MSG *pMsg);
/*****************************************************************************
   Parameters:
      pMsg: pointer to a message structure which will contain the received message
*****************************************************************************/

/*****************************************************************************
	MSK_CheckSum: Ask for checksum of active drive's memory.
*****************************************************************************/
bool MSK_CheckSum(WORD nStartAddr, WORD nEndAddr, WORD* pCheckSum);
/*****************************************************************************
   Parameters:
		nStartAddr:	start memory address
		nEndAddr: end memory address
		pCheckSum: received CheckSum from the drive
*****************************************************************************/

/*****************************************************************************
   MSK_SendData: This function transfers a block of data into active drive's memory
*****************************************************************************/
bool MSK_SendData(WORD *pData, WORD nAddr, BYTE memType, WORD nLength);
/*****************************************************************************
   Parameters:
        pData:      pointer to data buffer;
        nAddr:      memory address where the data will be stored
        memType:    memory space where the data will be stored (MSK_DM, MSK_PM OR MSK_SPI)
        nLength:    number of words to be stored
*****************************************************************************/

/*****************************************************************************
   MSK_ReceiveData: This function read a block of data from active drive's memory
*****************************************************************************/
bool MSK_ReceiveData(WORD *pData, WORD nAddr, BYTE memType, WORD nLength);
/*****************************************************************************
   Parameters:
      pData:      pointer to data buffer;
      nAddr:      memory address from where the data will be readed
      memType:    memory space from where the data will be readed (MSK_DM, MSK_PM OR MSK_SPI)
      nLength:    number of words to be readed
*****************************************************************************/

/*****************************************************************************
   MSK_SendBigData: This function transfers a block of data into active drive's memory,
		Note: the only difference from MSK_SendData is support for completion measurement
*****************************************************************************/
#ifdef __cplusplus
bool MSK_SendBigData(WORD *pData, WORD nAddr, BYTE memType, WORD nLength, MSK_Callback pf = NULL, void* pUsrData = NULL);
#else
bool MSK_SendBigData(WORD *pData, WORD nAddr, BYTE memType, WORD nLength, MSK_Callback pf, void* pUsrData);
#endif
/*****************************************************************************
   Parameters:
      pData:      pointer to data buffer;
      nAddr:      memory address where the data will be stored
      memType:    memory space where the data will be stored (MSK_DM, MSK_PM OR MSK_SPI)
      nLength:    number of words to be stored
      pf:         pointer to a callback function used to display progress status of this operation
      pUsrData:   pointer to user data passed to callback function
*****************************************************************************/

/*****************************************************************************
   MSK_ReceiveBigData: This function read a block of data from active drive's memory.
		Note: the only difference from MSK_ReceiveData is support for completion measurement
*****************************************************************************/
#ifdef __cplusplus
bool MSK_ReceiveBigData(WORD *pData, WORD nAddr, BYTE memType, WORD nLength, MSK_Callback pf = NULL, void* pUsrData = NULL);
#else
bool MSK_ReceiveBigData(WORD *pData, WORD nAddr, BYTE memType, WORD nLength, MSK_Callback pf, void* pUsrData);
#endif
/*****************************************************************************
   Parameters:
      pData:      pointer to data buffer;
      nAddr:      memory address from where the data will be readed
      memType:    memory space from where the data will be readed (MSK_DM, MSK_PM OR MSK_SPI)
      nLength:    number of words to be read
      pf:         pointer to a callback function used to display progress status of this operation
      pUsrData:   pointer to user data passed to callback function
*****************************************************************************/

/*****************************************************************************
   MSK_COFFDownload: download a COFF formatted file into drive's memory (PM or SPI)
*****************************************************************************/
#ifdef __cplusplus
bool MSK_COFFDownload(PCSTRING file_name, WORD* pEntry = NULL, WORD* pBeginAddr = NULL, WORD* pEndAddr = NULL,
                     MSK_Callback pf = NULL, void* pUsrData = NULL, WORD wSPISize = 0x4000);
#else
bool MSK_COFFDownload(PCSTRING file_name, WORD* pEntry, WORD* pBeginAddr, WORD* pEndAddr,
                     MSK_Callback pf, void* pUsrData, WORD wSPISize);
#endif
/*****************************************************************************
   Parameters:
      file_name:           name of the COFF file
      pEntry(out param):   will contain the entry point of the downloaded OUT (can be NULL)
      pBeginAddr(out param):   will contain the smallest location address written by download (can be NULL)
      pEndAddr(out param):   will contain the biggest location address written by download (can be NULL)
      pf:                  pointer to a callback function used to control the download process (can be NULL)
      pUsrData:            user data passed to callback function (can be NULL)
      wSPISize:            size (in WORDS) of the SPI memory. Used only for testing if a section is out of memory.
                              The range is between 0 and 0x4000.
*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* tmlcomm.h */
