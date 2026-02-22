/*
 * tmlSerial.h
 *
 * Native serial protocol implementation for Technosoft TML drives.
 * Replaces the proprietary TML_lib / tmlcomm binary libraries with
 * a direct POSIX serial (RS-232/RS-485) implementation.
 *
 * Protocol reference:
 *   https://www.technosoftmotion.com/ESM-um-html/communication_protocol_serial.htm
 *
 * Author:  Andrea Michelotti — INFN-LNF
 * Date:    2026-02
 */

#ifndef TML_SERIAL_H
#define TML_SERIAL_H

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <string>
#include <map>
#include <mutex>
#include <termios.h>

/* ================================================================= */
/*                     Types matching TML_lib                        */
/* ================================================================= */

#ifndef BYTE
typedef uint8_t  BYTE;
#endif
#ifndef WORD
typedef uint16_t WORD;
#endif
#ifndef DWORD
typedef uint32_t DWORD;
#endif
#ifndef BOOL
typedef int BOOL;
#endif
#ifndef TRUE
#define TRUE  1
#endif
#ifndef FALSE
#define FALSE 0
#endif

typedef const char* LPCSTR;
typedef char*       LPSTR;

/* ================================================================= */
/*                     TML Protocol Constants                        */
/* ================================================================= */

/* Channel types (same values as TML_lib) */
#define CHANNEL_RS232    0
#define CHANNEL_RS485    1
#define CHANNEL_TCP      2   /* TCP/IP (XPORT, ser2net, etc.) */
#define CHANNEL_XPORT_IP 16  /* Digi XPORT — mapped to CHANNEL_TCP internally */

/* Register selection indices for readStatus() */
#define REG_MCR  0
#define REG_MSR  1
#define REG_ISR  2
#define REG_SRL  3
#define REG_SRH  4
#define REG_MER  5

/* Motion update moments */
#define UPDATE_NONE       -1
#define UPDATE_ON_EVENT    0
#define UPDATE_IMMEDIATE   1

/* Reference base */
#define FROM_MEASURE    0
#define FROM_REFERENCE  1

/* Limit switch types */
#define LSW_NEGATIVE  -1
#define LSW_POSITIVE   1

/* Transition types */
#define TRANSITION_HIGH_TO_LOW  -1
#define TRANSITION_DISABLE       0
#define TRANSITION_LOW_TO_HIGH   1

/* ================================================================= */
/*      TML Well-Known Data Memory Addresses                         */
/*  (Standard across all TML-based Technosoft drives)                */
/* ================================================================= */

/* Position / speed / acceleration command registers (32-bit) */
#define TML_DM_APOS       0x0228   /* Actual (measured) position (32-bit, low word addr) */
#define TML_DM_TPOS       0x022A   /* Target position TPOS (32-bit, low word addr) */
#define TML_DM_CPOS       0x022E   /* Commanded position reference (32-bit) */
#define TML_DM_CSPD       0x0230   /* Commanded speed (32-bit, IU fixed 16.16) */
#define TML_DM_CACC       0x0232   /* Commanded acceleration (32-bit, IU fixed 16.16) */

/* Status / control registers (16-bit) */
#define TML_DM_MCR        0x0908   /* Motion Control Register */
#define TML_DM_MSR        0x0909   /* Motion Status Register */
#define TML_DM_ISR        0x090A   /* Interrupt Status Register */
#define TML_DM_SRL        0x090E   /* Status Register Low */
#define TML_DM_SRH        0x090F   /* Status Register High */
#define TML_DM_MER        0x0910   /* Motion Error Register */

/* Event masks */
#define TML_DM_SRL_MASK   0x0912   /* SRL event mask */
#define TML_DM_SRH_MASK   0x0913   /* SRH event mask */
#define TML_DM_MER_MASK   0x0914   /* MER event mask */

/* Master ID for unsolicited messages */
#define TML_DM_MASTERID   0x0916   /* MASTERID register */

/* ================================================================= */
/*      TML Online Communication Opcodes                             */
/*  (Well-documented: serial protocol reference)                     */
/* ================================================================= */

#define TML_OP_GIVE_ME_DATA_16   0xB004  /* Request 16-bit read from drive */
#define TML_OP_GIVE_ME_DATA_32   0xB005  /* Request 32-bit read from drive */
#define TML_OP_TAKE_DATA_16      0xB404  /* Response: 16-bit data from drive */
#define TML_OP_TAKE_DATA_32      0xB405  /* Response: 32-bit data from drive */

/* ================================================================= */
/*      TML Instruction Opcodes                                      */
/*  (Binary TML instruction codes — Technosoft TML ISA)              */
/* ================================================================= */

/*
 * Online communication protocol: Write 16-bit variable to DM address.
 *   OpCode = 0x9004
 *   Data[0] = DM address, Data[1] = value
 *   2 data words
 * (Verified from TML_lib libtmlcomm.so SendData @ 0xa47c)
 */
#define TML_OP_WRITE_DATA_16     0x9004

/*
 * Online communication protocol: Write 32-bit variable to DM address.
 *   OpCode = 0x9005
 *   Data[0] = DM address, Data[1] = value_low, Data[2] = value_high
 *   3 data words
 * (Verified from TML_lib libtmlcomm.so SendData @ 0xa47c)
 */
#define TML_OP_WRITE_DATA_32     0x9005

/*
 * Simple control commands (no data words):
 *   Sent as single-opcode TML instructions
 */
#define TML_OP_ENDINIT    0x0800  /* End initialization (validate setup) */
#define TML_OP_AXISON     0x6B01  /* Enable power stage */
#define TML_OP_AXISOFF    0x6B00  /* Disable power stage */
#define TML_OP_STOP       0x6C00  /* Decelerated stop (STOP0) */
#define TML_OP_STOP_IMM   0x6C01  /* Immediate stop (STOP0!) */
#define TML_OP_FAULTR     0x6B02  /* Reset faults */
#define TML_OP_UPD        0x6900  /* Update on event */
#define TML_OP_UPD_IMM    0x6901  /* Update immediate */
#define TML_OP_ABORT      0x6B04  /* Abort cancelable CALL */
#define TML_OP_CPA        0x6B05  /* Copy actual position (absolute ref) */
#define TML_OP_CPR        0x6B06  /* Copy actual position (relative ref) */
#define TML_OP_SAVE       0x6C04  /* Save parameters to EEPROM */
#define TML_OP_RESET      0x6C05  /* Reset drive */
#define TML_OP_STA        0x6908  /* Start (used with events) */

/*
 * Mode selection commands:
 *   MODE PP  = position profile (trapezoidal)
 *   MODE SP  = speed profile
 *   MODE PSC = position S-curve
 * These encode the motion mode into MCR via TML instruction.
 */
#define TML_OP_MODE_PP    0x6040  /* Mode: Position Profile */
#define TML_OP_MODE_SP    0x6041  /* Mode: Speed Profile */

/* ================================================================= */
/*      MCR Bit Definitions                                          */
/* ================================================================= */

#define MCR_BIT_UPDATE    (1 << 8)   /* Update request (self-clearing) */
#define MCR_BIT_AXISON    (1 << 15)  /* Power stage enabled */

/* Motion mode bits [3:0] of MCR */
#define MCR_MODE_PP       0x0001     /* Position profile (trapezoidal) */
#define MCR_MODE_SP       0x0002     /* Speed profile */
#define MCR_MODE_PSC      0x0003     /* Position S-curve */
#define MCR_MODE_MASK     0x000F     /* Mode bits mask */

/* Reference bits */
#define MCR_BIT_REF_BASE  (1 << 5)  /* 0=FROM_MEASURE, 1=FROM_REFERENCE */
#define MCR_BIT_RELATIVE  (1 << 7)  /* 0=absolute, 1=relative */

/* ================================================================= */
/*      Serial Protocol Constants                                    */
/* ================================================================= */

#define TML_ACK_BYTE       0x4F   /* 'O' — acknowledge OK */
#define TML_SYNC_BYTE      0x0D   /* Synchronization byte */
#define TML_MAX_SYNC_RETRY 15     /* Max SYNC retries */
#define TML_ACK_TIMEOUT_MS 100    /* Timeout for ACK (ms) */
#define TML_RESP_TIMEOUT_MS 500   /* Timeout for response message (ms) */
#define TML_MAX_MSG_BYTES  16     /* Max serial message bytes (1+2+2+8+1) */

/* ================================================================= */
/*      TML Serial Message (internal representation)                 */
/* ================================================================= */

struct TmlMsg {
    WORD addr;          /* Destination axis: axisID << 4 */
    WORD opCode;        /* TML instruction opcode */
    int  nData;         /* Number of 16-bit data words (0..4) */
    WORD data[4];       /* Data words */
};

/* ================================================================= */
/*      Variable Address Map                                         */
/* ================================================================= */

/**
 * Holds the mapping from TML variable names to DM addresses.
 * Populated with well-known defaults; can be overridden.
 */
class TmlVariableMap {
public:
    TmlVariableMap();

    /** Look up address by name. Returns true if found. */
    bool getAddress(const char *name, WORD &addr) const;

    /** Set/override an address mapping */
    void setAddress(const char *name, WORD addr);

    /** Register the well-known default addresses */
    void registerDefaults();

private:
    std::map<std::string, WORD> map_;
};

/* ================================================================= */
/*      TmlChannel — one serial communication channel                */
/* ================================================================= */

class TmlChannel {
public:
    TmlChannel();
    ~TmlChannel();

    /**
     * Open a serial port or TCP socket for TML communication.
     * @param devPath   Serial device "/dev/ttyUSB0" or TCP "host:port"
     * @param hostId    Host address on the bus (1-255, typically 255 for RS-232)
     * @param baudRate  9600..115200 (ignored for TCP)
     * @param channelType CHANNEL_RS232, CHANNEL_RS485, or CHANNEL_TCP
     *                    (auto-detected from devPath if RS232 and contains ':')
     * @return file descriptor (>= 0) on success, -1 on error
     */
    int open(const char *devPath, BYTE hostId, DWORD baudRate,
             BYTE channelType = CHANNEL_RS232);

    /** @return true if this channel uses TCP/IP transport */
    bool isTcp() const { return isTcp_; }

    /** Close the serial port. */
    void close();

    /** @return true if the channel is open */
    bool isOpen() const { return fd_ >= 0; }

    /** @return file descriptor */
    int fd() const { return fd_; }

    /** @return host axis ID */
    BYTE hostId() const { return hostId_; }

    /** @return the variable map (for address resolution) */
    TmlVariableMap &varMap() { return varMap_; }

    /* ---- Low-level message send/receive ---- */

    /**
     * Send a TML message and wait for ACK.
     * @param msg  The message to send
     * @return true if ACK received OK
     */
    bool sendMessage(const TmlMsg &msg);

    /**
     * Receive a TML message (response to a Type B request).
     * @param msg  Output: the received message
     * @param timeoutMs Timeout in milliseconds
     * @return true if a valid message was received
     */
    bool receiveMessage(TmlMsg &msg, int timeoutMs = TML_RESP_TIMEOUT_MS);

    /**
     * Attempt to re-synchronize the serial link by sending SYNC bytes.
     * @return true if synchronization was restored
     */
    bool resync();

    /* ---- High-level TML operations ---- */

    /**
     * Send a simple TML command (no data words) to the active axis.
     */
    bool sendCommand(WORD opCode);

    /**
     * Write a 16-bit value to a DM address on the active axis.
     */
    bool writeData16(WORD address, WORD value);

    /**
     * Write a 32-bit value to a DM address on the active axis.
     * (low word at address, high word at address+1)
     */
    bool writeData32(WORD address, uint32_t value);

    /**
     * Read a 16-bit value from a DM address on the active axis.
     * Uses GiveMeData/TakeData protocol.
     */
    bool readData16(WORD address, WORD &value);

    /**
     * Read a 32-bit value from a DM address on the active axis.
     * Uses GiveMeData32/TakeData32 protocol.
     */
    bool readData32(WORD address, uint32_t &value);

    /** Select the active axis for subsequent operations */
    void selectAxis(BYTE axisId);

    /** Get the currently selected axis */
    BYTE activeAxis() const { return activeAxisId_; }

    /** Get the last error text */
    const char *lastError() const { return lastError_; }

    /** Dump raw bytes as hex string for trace output */
    static void hexDump(const char *tag, const uint8_t *buf, int len);

private:
    int  fd_;               /* Serial port / socket file descriptor */
    BYTE hostId_;           /* Host axis ID */
    BYTE activeAxisId_;     /* Currently selected drive axis ID */
    BYTE channelType_;      /* RS-232, RS-485, or TCP */
    bool isTcp_;            /* true when using TCP socket transport */
    char lastError_[256];   /* Last error description */
    TmlVariableMap varMap_; /* Variable name → address mapping */

    /* Serialise a TmlMsg into wire bytes. Returns total byte count. */
    int serialiseMessage(const TmlMsg &msg, uint8_t *buf, size_t bufLen);

    /* Deserialise wire bytes into a TmlMsg. Returns true on success. */
    bool deserialiseMessage(const uint8_t *buf, int len, TmlMsg &msg);

    /* Compute checksum: sum modulo 256 of all message bytes except checksum */
    static uint8_t checksum(const uint8_t *buf, int len);

    /* Read exactly n bytes with timeout. Returns bytes read. */
    int readBytes(uint8_t *buf, int count, int timeoutMs);

    /* Write all bytes. Returns true if all written. */
    bool writeBytes(const uint8_t *buf, int count);

    /* Wait for ACK byte. Returns true if ACK received. */
    bool waitAck(int timeoutMs = TML_ACK_TIMEOUT_MS);

    /* Set error message */
    void setError(const char *fmt, ...);

    /* Configure termios for TML serial communication */
    bool configurePort(DWORD baudRate);

    /* Open a TCP socket to host:port. Returns fd or -1. */
    int connectTcp(const char *hostPort);

    /* Detect if devPath looks like IP:port */
    static bool looksLikeTcp(const char *devPath);

    /* Map baud rate integer to termios speed constant */
    static speed_t baudToSpeed(DWORD baud);
};

/* ================================================================= */
/*      TML_lib-Compatible API                                       */
/*  Drop-in replacements for the TML_lib functions used by the       */
/*  EPICS motor driver.  These maintain internal state (active        */
/*  channel, active axis) just like the original library.            */
/* ================================================================= */

#ifdef __cplusplus
extern "C" {
#endif

LPCSTR TS_GetLastErrorText(void);

int  TS_OpenChannel(LPCSTR pszDevName, BYTE btType, BYTE nHostID, DWORD baudrate);
BOOL TS_SelectChannel(int fd);
void TS_CloseChannel(int fd);

int  TS_LoadSetup(LPCSTR setupPath);
BOOL TS_SetupAxis(BYTE axisID, int idxSetup);
BOOL TS_SelectAxis(BYTE axisID);

BOOL TS_DriveInitialisation(void);
BOOL TS_Power(BOOL Enable);
BOOL TS_ReadStatus(short SelIndex, WORD &Status);
BOOL TS_SetPosition(long PosValue);
BOOL TS_Stop(void);
BOOL TS_ABORT(void);
BOOL TS_ResetFault(void);

BOOL TS_GetLongVariable(LPCSTR pszName, long &value);
BOOL TS_SetLongVariable(LPCSTR pszName, long value);
BOOL TS_GetIntVariable(LPCSTR pszName, short &value);
BOOL TS_SetIntVariable(LPCSTR pszName, short value);
BOOL TS_GetFixedVariable(LPCSTR pszName, double &value);
BOOL TS_SetFixedVariable(LPCSTR pszName, double value);

BOOL TS_MoveAbsolute(long AbsPosition, double Speed, double Acceleration,
                     short MoveMoment, short ReferenceBase);
BOOL TS_MoveRelative(long RelPosition, double Speed, double Acceleration,
                     BOOL IsAdditive, short MoveMoment, short ReferenceBase);
BOOL TS_MoveVelocity(double Speed, double Acceleration,
                     short MoveMoment, short ReferenceBase);

BOOL TS_SetEventOnMotionComplete(BOOL WaitEvent, BOOL EnableStop);
BOOL TS_SetEventOnLimitSwitch(short LSWType, short TransitionType,
                              BOOL WaitEvent, BOOL EnableStop);

BOOL TS_UpdateImmediate(void);
BOOL TS_Save(void);
BOOL TS_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* TML_SERIAL_H */
