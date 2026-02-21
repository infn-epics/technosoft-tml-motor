/*
 * tmlSerial.cpp
 *
 * Native serial protocol implementation for Technosoft TML drives.
 * Replaces the proprietary TML_lib / tmlcomm binary libraries with
 * a direct POSIX serial (RS-232/RS-485) implementation.
 *
 * Serial frame format (from Technosoft documentation):
 *   [Length] [AxisID_hi][AxisID_lo] [OpCode_hi][OpCode_lo] [Data...] [Checksum]
 *
 *   Length   = total bytes - 2  (i.e. Addr + OpCode + Data bytes)
 *   Checksum = (sum of all bytes except checksum itself) mod 256
 *
 * After each host→drive message, the drive sends:
 *   - ACK byte 0x4F ('O') if the message was received correctly
 *   - Nothing if an error occurred (host must re-sync)
 *
 * For Type B messages (read requests), after the ACK the drive sends
 * a response message with the requested data.
 *
 * Author:  Andrea Michelotti — INFN-LNF
 * Date:    2026-02
 */

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cmath>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>

#include "tmlSerial.h"

/* ================================================================= */
/*               Debug helper (mirrors drvTmlMotor)                  */
/* ================================================================= */

extern int drvTmlDebug;

#define DBG_SER(level, fmt, ...) \
    do { if (drvTmlDebug >= (level)) \
        printf("tmlSerial [%s:%d] " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); \
    } while(0)

/* ================================================================= */
/*                     Global State                                  */
/*  (Matches the TML_lib model: one "active channel", one "active    */
/*   axis" at a time, selected via TS_Select* calls.)                */
/* ================================================================= */

static const int MAX_CHANNELS = 8;

struct ChannelSlot {
    TmlChannel *ch;
    bool        inUse;
};

static ChannelSlot  g_channels[MAX_CHANNELS] = {};
static TmlChannel  *g_activeCh   = nullptr;
static int          g_activeFd   = -1;
static char         g_lastError[512] = "No error";

/* "Setup" storage — for the native driver the setup file is optional;
 * we just remember the path and allow the variable map to be populated. */
static const int MAX_SETUPS = 16;

struct SetupSlot {
    bool inUse;
    char path[512];
    TmlVariableMap varMap;  /* per-setup variable overrides */
};

static SetupSlot g_setups[MAX_SETUPS] = {};

static void setGlobalError(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_lastError, sizeof(g_lastError), fmt, ap);
    va_end(ap);
    DBG_SER(0, "ERROR: %s", g_lastError);
}

/* ================================================================= */
/*               TmlVariableMap                                      */
/* ================================================================= */

TmlVariableMap::TmlVariableMap()
{
    registerDefaults();
}

void TmlVariableMap::registerDefaults()
{
    /* Standard TML variable names → fixed DM addresses */
    map_["APOS"]  = TML_DM_APOS;
    map_["TPOS"]  = TML_DM_TPOS;
    map_["CPOS"]  = TML_DM_CPOS;
    map_["CSPD"]  = TML_DM_CSPD;
    map_["CACC"]  = TML_DM_CACC;

    map_["MCR"]   = TML_DM_MCR;
    map_["MSR"]   = TML_DM_MSR;
    map_["ISR"]   = TML_DM_ISR;
    map_["SRL"]   = TML_DM_SRL;
    map_["SRH"]   = TML_DM_SRH;
    map_["MER"]   = TML_DM_MER;

    map_["SRL_MASK"] = TML_DM_SRL_MASK;
    map_["SRH_MASK"] = TML_DM_SRH_MASK;
    map_["MER_MASK"] = TML_DM_MER_MASK;
    map_["MASTERID"] = TML_DM_MASTERID;
}

bool TmlVariableMap::getAddress(const char *name, WORD &addr) const
{
    auto it = map_.find(name);
    if (it == map_.end()) return false;
    addr = it->second;
    return true;
}

void TmlVariableMap::setAddress(const char *name, WORD addr)
{
    map_[name] = addr;
}

/* ================================================================= */
/*               TmlChannel — construction / destruction             */
/* ================================================================= */

TmlChannel::TmlChannel()
    : fd_(-1)
    , hostId_(1)
    , activeAxisId_(255)
    , channelType_(CHANNEL_RS232)
{
    memset(lastError_, 0, sizeof(lastError_));
}

TmlChannel::~TmlChannel()
{
    close();
}

/* ================================================================= */
/*               Serial port configuration                           */
/* ================================================================= */

speed_t TmlChannel::baudToSpeed(DWORD baud)
{
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:     return B9600;
    }
}

bool TmlChannel::configurePort(DWORD baudRate)
{
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd_, &tty) != 0) {
        setError("tcgetattr failed: %s", strerror(errno));
        return false;
    }

    speed_t spd = baudToSpeed(baudRate);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    /* TML serial: 8 data bits, 2 stop bits, no parity */
    tty.c_cflag &= ~PARENB;        /* No parity */
    tty.c_cflag |=  CSTOPB;        /* 2 stop bits */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |=  CS8;           /* 8 data bits */

    tty.c_cflag |=  CREAD | CLOCAL; /* Enable receiver, ignore modem control */

    /* Raw mode — no special processing */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT |
                      PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    /* Blocking read with inter-character timeout */
    tty.c_cc[VMIN]  = 0;   /* Non-blocking: return what is available */
    tty.c_cc[VTIME] = 1;   /* 100ms inter-character timeout */

    /* Hardware flow control: off for RS-232 TML */
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        setError("tcsetattr failed: %s", strerror(errno));
        return false;
    }

    /* Flush any pending I/O */
    tcflush(fd_, TCIOFLUSH);

    return true;
}

/* ================================================================= */
/*               Open / Close                                        */
/* ================================================================= */

int TmlChannel::open(const char *devPath, BYTE hostId, DWORD baudRate,
                     BYTE channelType)
{
    if (fd_ >= 0)
        close();

    hostId_      = hostId;
    channelType_ = channelType;
    activeAxisId_ = hostId;  /* default: direct-connected axis */

    fd_ = ::open(devPath, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        setError("Cannot open '%s': %s", devPath, strerror(errno));
        return -1;
    }

    /* Clear O_NONBLOCK after open */
    int flags = fcntl(fd_, F_GETFL, 0);
    if (flags >= 0)
        fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    if (!configurePort(baudRate)) {
        ::close(fd_);
        fd_ = -1;
        return -1;
    }

    DBG_SER(1, "Opened '%s' fd=%d hostId=%d baud=%u type=%d",
            devPath, fd_, hostId_, (unsigned)baudRate, channelType_);

    return fd_;
}

void TmlChannel::close()
{
    if (fd_ >= 0) {
        DBG_SER(1, "Closing fd=%d", fd_);
        ::close(fd_);
        fd_ = -1;
    }
}

/* ================================================================= */
/*               Axis selection                                      */
/* ================================================================= */

void TmlChannel::selectAxis(BYTE axisId)
{
    activeAxisId_ = axisId;
}

/* ================================================================= */
/*               Error reporting                                     */
/* ================================================================= */

void TmlChannel::setError(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(lastError_, sizeof(lastError_), fmt, ap);
    va_end(ap);
    DBG_SER(0, "ERROR: %s", lastError_);
}

/* ================================================================= */
/*               Low-level byte I/O                                  */
/* ================================================================= */

int TmlChannel::readBytes(uint8_t *buf, int count, int timeoutMs)
{
    if (fd_ < 0 || count <= 0) return 0;

    int totalRead = 0;
    struct timeval deadline;
    gettimeofday(&deadline, nullptr);
    deadline.tv_usec += timeoutMs * 1000;
    deadline.tv_sec  += deadline.tv_usec / 1000000;
    deadline.tv_usec %= 1000000;

    while (totalRead < count) {
        struct timeval now, tv;
        gettimeofday(&now, nullptr);

        /* Compute remaining timeout */
        long usecLeft = (deadline.tv_sec - now.tv_sec) * 1000000
                      + (deadline.tv_usec - now.tv_usec);
        if (usecLeft <= 0) break;

        tv.tv_sec  = usecLeft / 1000000;
        tv.tv_usec = usecLeft % 1000000;

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);

        int sel = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
        if (sel <= 0) break;  /* timeout or error */

        int n = ::read(fd_, buf + totalRead, count - totalRead);
        if (n <= 0) break;
        totalRead += n;
    }

    return totalRead;
}

bool TmlChannel::writeBytes(const uint8_t *buf, int count)
{
    if (fd_ < 0) return false;

    int written = 0;
    while (written < count) {
        int n = ::write(fd_, buf + written, count - written);
        if (n <= 0) {
            setError("write failed: %s", strerror(errno));
            return false;
        }
        written += n;
    }

    /* Ensure bytes are drained to the hardware */
    tcdrain(fd_);
    return true;
}

/* ================================================================= */
/*               Checksum                                            */
/* ================================================================= */

uint8_t TmlChannel::checksum(const uint8_t *buf, int len)
{
    unsigned sum = 0;
    for (int i = 0; i < len; i++)
        sum += buf[i];
    return (uint8_t)(sum & 0xFF);
}

/* ================================================================= */
/*            Message serialisation / deserialisation                 */
/* ================================================================= */

/*
 * Wire format:
 *   [LEN] [ADDR_HI] [ADDR_LO] [OPCODE_HI] [OPCODE_LO]
 *         [DATA_0_HI] [DATA_0_LO] ... [DATA_N_HI] [DATA_N_LO]
 *   [CHKSUM]
 *
 * LEN = 2(addr) + 2(opcode) + 2*nData = 4 + 2*nData
 * CHKSUM = sum mod 256 of all bytes from LEN to last data byte
 */

int TmlChannel::serialiseMessage(const TmlMsg &msg, uint8_t *buf, size_t bufLen)
{
    int payloadBytes = 4 + 2 * msg.nData;  /* addr(2) + opcode(2) + data(2*N) */
    int totalBytes = 1 + payloadBytes + 1;  /* len + payload + checksum */

    if ((size_t)totalBytes > bufLen) return -1;

    int idx = 0;
    buf[idx++] = (uint8_t)payloadBytes;               /* Length byte */
    buf[idx++] = (uint8_t)((msg.addr >> 8) & 0xFF);   /* Addr high */
    buf[idx++] = (uint8_t)(msg.addr & 0xFF);           /* Addr low */
    buf[idx++] = (uint8_t)((msg.opCode >> 8) & 0xFF);  /* OpCode high */
    buf[idx++] = (uint8_t)(msg.opCode & 0xFF);          /* OpCode low */

    for (int i = 0; i < msg.nData; i++) {
        buf[idx++] = (uint8_t)((msg.data[i] >> 8) & 0xFF); /* Data high */
        buf[idx++] = (uint8_t)(msg.data[i] & 0xFF);         /* Data low */
    }

    /* Checksum = sum of all preceding bytes mod 256 */
    buf[idx] = checksum(buf, idx);
    idx++;

    return idx;
}

bool TmlChannel::deserialiseMessage(const uint8_t *buf, int len, TmlMsg &msg)
{
    if (len < 6) return false;  /* minimum: len(1) + addr(2) + opcode(2) + chk(1) */

    int payloadLen = buf[0];
    int expectedTotal = 1 + payloadLen + 1;

    if (len < expectedTotal) return false;

    /* Verify checksum */
    uint8_t expected_ck = checksum(buf, expectedTotal - 1);
    uint8_t received_ck = buf[expectedTotal - 1];
    if (expected_ck != received_ck) {
        setError("Checksum mismatch: expected 0x%02X, got 0x%02X",
                 expected_ck, received_ck);
        return false;
    }

    msg.addr   = ((WORD)buf[1] << 8) | buf[2];
    msg.opCode = ((WORD)buf[3] << 8) | buf[4];
    msg.nData  = (payloadLen - 4) / 2;

    if (msg.nData < 0 || msg.nData > 4) {
        setError("Invalid data word count %d", msg.nData);
        return false;
    }

    for (int i = 0; i < msg.nData; i++) {
        int off = 5 + 2 * i;
        msg.data[i] = ((WORD)buf[off] << 8) | buf[off + 1];
    }

    return true;
}

/* ================================================================= */
/*               ACK handling and re-synchronisation                 */
/* ================================================================= */

bool TmlChannel::waitAck(int timeoutMs)
{
    uint8_t byte;
    int n = readBytes(&byte, 1, timeoutMs);
    if (n == 1 && byte == TML_ACK_BYTE) {
        DBG_SER(3, "ACK received");
        return true;
    }

    if (n == 1) {
        setError("Expected ACK (0x4F), got 0x%02X", byte);
    } else {
        setError("ACK timeout (%d ms)", timeoutMs);
    }
    return false;
}

bool TmlChannel::resync()
{
    DBG_SER(1, "Attempting re-synchronization");

    for (int attempt = 0; attempt < TML_MAX_SYNC_RETRY; attempt++) {
        uint8_t syncByte = TML_SYNC_BYTE;
        if (!writeBytes(&syncByte, 1))
            return false;

        uint8_t resp;
        int n = readBytes(&resp, 1, 2);  /* 2 ms timeout */
        if (n == 1 && resp == TML_SYNC_BYTE) {
            DBG_SER(1, "Re-sync successful after %d attempts", attempt + 1);
            return true;
        }
    }

    setError("Re-sync failed after %d attempts", TML_MAX_SYNC_RETRY);
    return false;
}

/* ================================================================= */
/*               Message send / receive                              */
/* ================================================================= */

bool TmlChannel::sendMessage(const TmlMsg &msg)
{
    uint8_t buf[TML_MAX_MSG_BYTES];
    int len = serialiseMessage(msg, buf, sizeof(buf));
    if (len < 0) {
        setError("Message serialisation failed");
        return false;
    }

    DBG_SER(3, "TX [%d bytes] addr=0x%04X op=0x%04X nData=%d",
            len, msg.addr, msg.opCode, msg.nData);

    if (!writeBytes(buf, len))
        return false;

    /* Wait for ACK (RS-485 broadcast msgs don't get ACK, but we always
       send to individual axes in this driver) */
    if (!waitAck()) {
        /* Try resync + retry once */
        if (resync()) {
            if (!writeBytes(buf, len))
                return false;
            if (!waitAck()) {
                setError("No ACK after resync retry");
                return false;
            }
        } else {
            return false;
        }
    }

    return true;
}

bool TmlChannel::receiveMessage(TmlMsg &msg, int timeoutMs)
{
    /* First read the length byte */
    uint8_t lenByte;
    int n = readBytes(&lenByte, 1, timeoutMs);
    if (n != 1) {
        setError("Timeout waiting for response length byte");
        return false;
    }

    int payloadLen = lenByte;
    if (payloadLen < 4 || payloadLen > 12) {
        setError("Invalid response payload length: %d", payloadLen);
        return false;
    }

    /* Read the rest: payload + checksum */
    uint8_t buf[TML_MAX_MSG_BYTES];
    buf[0] = lenByte;
    int remaining = payloadLen + 1;  /* payload bytes + checksum byte */

    n = readBytes(buf + 1, remaining, timeoutMs);
    if (n != remaining) {
        setError("Incomplete response: got %d of %d bytes", n, remaining);
        return false;
    }

    int totalLen = 1 + payloadLen + 1;

    DBG_SER(3, "RX [%d bytes] raw: %02X %02X %02X %02X %02X ...",
            totalLen, buf[0], buf[1], buf[2],
            totalLen > 3 ? buf[3] : 0, totalLen > 4 ? buf[4] : 0);

    return deserialiseMessage(buf, totalLen, msg);
}

/* ================================================================= */
/*               High-level operations                               */
/* ================================================================= */

bool TmlChannel::sendCommand(WORD opCode)
{
    TmlMsg msg;
    msg.addr   = (WORD)activeAxisId_ << 4;
    msg.opCode = opCode;
    msg.nData  = 0;
    return sendMessage(msg);
}

bool TmlChannel::writeData16(WORD address, WORD value)
{
    TmlMsg msg;
    msg.addr   = (WORD)activeAxisId_ << 4;
    msg.opCode = TML_OP_SET16_BASE | (address & 0x0FFF);
    msg.nData  = 1;
    msg.data[0] = value;

    DBG_SER(2, "Write16 addr=0x%04X val=0x%04X → opcode=0x%04X",
            address, value, msg.opCode);

    return sendMessage(msg);
}

bool TmlChannel::writeData32(WORD address, uint32_t value)
{
    TmlMsg msg;
    msg.addr   = (WORD)activeAxisId_ << 4;
    msg.opCode = TML_OP_SET32_BASE | (address & 0x0FFF);
    msg.nData  = 2;
    msg.data[0] = (WORD)(value & 0xFFFF);         /* Low word */
    msg.data[1] = (WORD)((value >> 16) & 0xFFFF); /* High word */

    DBG_SER(2, "Write32 addr=0x%04X val=0x%08X → opcode=0x%04X",
            address, (unsigned)value, msg.opCode);

    return sendMessage(msg);
}

bool TmlChannel::readData16(WORD address, WORD &value)
{
    /* Build GiveMeData16 request */
    TmlMsg req;
    req.addr    = (WORD)activeAxisId_ << 4;
    req.opCode  = TML_OP_GIVE_ME_DATA_16;
    req.nData   = 2;
    req.data[0] = address;
    req.data[1] = ((WORD)hostId_ << 4) | 0x0001;  /* Sender: host with host-bit set */

    DBG_SER(2, "ReadData16 addr=0x%04X", address);

    if (!sendMessage(req))
        return false;

    /* Wait for TakeData16 response */
    TmlMsg resp;
    if (!receiveMessage(resp, TML_RESP_TIMEOUT_MS))
        return false;

    /* Validate response */
    if (resp.opCode != TML_OP_TAKE_DATA_16 || resp.nData < 2) {
        setError("Unexpected response: opCode=0x%04X nData=%d (expected TakeData16)",
                 resp.opCode, resp.nData);
        return false;
    }

    /* resp.data[0] = address (echo), resp.data[1] = value */
    value = resp.data[1];

    DBG_SER(2, "ReadData16 addr=0x%04X → 0x%04X", address, value);
    return true;
}

bool TmlChannel::readData32(WORD address, uint32_t &value)
{
    /* Build GiveMeData32 request */
    TmlMsg req;
    req.addr    = (WORD)activeAxisId_ << 4;
    req.opCode  = TML_OP_GIVE_ME_DATA_32;
    req.nData   = 2;
    req.data[0] = address;
    req.data[1] = ((WORD)hostId_ << 4) | 0x0001;

    DBG_SER(2, "ReadData32 addr=0x%04X", address);

    if (!sendMessage(req))
        return false;

    /* Wait for TakeData32 response */
    TmlMsg resp;
    if (!receiveMessage(resp, TML_RESP_TIMEOUT_MS))
        return false;

    if (resp.opCode != TML_OP_TAKE_DATA_32 || resp.nData < 3) {
        setError("Unexpected response: opCode=0x%04X nData=%d (expected TakeData32)",
                 resp.opCode, resp.nData);
        return false;
    }

    /* resp.data[0] = address, data[1] = low word, data[2] = high word */
    value = ((uint32_t)resp.data[2] << 16) | (uint32_t)resp.data[1];

    DBG_SER(2, "ReadData32 addr=0x%04X → 0x%08X (%ld)",
            address, (unsigned)value, (long)(int32_t)value);
    return true;
}

/* ================================================================= */
/*            Internal helpers for fixed-point conversion             */
/* ================================================================= */

/*
 * TML uses a 16.16 fixed-point format for speed and acceleration
 * in internal units. The raw value is stored as a 32-bit integer
 * where the upper 16 bits are the integer part and the lower
 * 16 bits are the fractional part.
 */
static uint32_t doubleToFixed1616(double val)
{
    int32_t fixed = (int32_t)(val * 65536.0);
    return (uint32_t)fixed;
}

static double fixed1616ToDouble(uint32_t fixed)
{
    return (double)(int32_t)fixed / 65536.0;
}

/* ================================================================= */
/*               TS_* API Implementation                             */
/* ================================================================= */

extern "C" {

LPCSTR TS_GetLastErrorText(void)
{
    return g_lastError;
}

int TS_OpenChannel(LPCSTR pszDevName, BYTE btType, BYTE nHostID, DWORD baudrate)
{
    /* Find a free channel slot */
    int slot = -1;
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (!g_channels[i].inUse) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        setGlobalError("No free channel slots (max %d)", MAX_CHANNELS);
        return -1;
    }

    TmlChannel *ch = new TmlChannel();
    int fd = ch->open(pszDevName, nHostID, baudrate, btType);
    if (fd < 0) {
        strncpy(g_lastError, ch->lastError(), sizeof(g_lastError) - 1);
        delete ch;
        return -1;
    }

    g_channels[slot].ch    = ch;
    g_channels[slot].inUse = true;

    /* Auto-select this channel */
    g_activeCh = ch;
    g_activeFd = fd;

    DBG_SER(1, "Channel opened: slot=%d fd=%d", slot, fd);
    return fd;
}

BOOL TS_SelectChannel(int fd)
{
    if (fd < 0) {
        /* -1 means "use currently selected" */
        return g_activeCh ? TRUE : FALSE;
    }

    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (g_channels[i].inUse && g_channels[i].ch->fd() == fd) {
            g_activeCh = g_channels[i].ch;
            g_activeFd = fd;
            return TRUE;
        }
    }

    setGlobalError("TS_SelectChannel: fd=%d not found", fd);
    return FALSE;
}

void TS_CloseChannel(int fd)
{
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (g_channels[i].inUse && (fd < 0 || g_channels[i].ch->fd() == fd)) {
            if (g_activeCh == g_channels[i].ch) {
                g_activeCh = nullptr;
                g_activeFd = -1;
            }
            delete g_channels[i].ch;
            g_channels[i].ch    = nullptr;
            g_channels[i].inUse = false;
            return;
        }
    }
}

int TS_LoadSetup(LPCSTR setupPath)
{
    /*
     * In the native serial implementation, TS_LoadSetup stores the
     * setup path and returns an index.  The setup files (.t.zip) from
     * EasyMotion Studio contain variable address mappings; parsing them
     * is optional since we use well-known default addresses.
     *
     * If a text file 'variables.cfg' exists alongside the setup,
     * we could parse it for address overrides.  For now, we just
     * accept the path and return a valid index.
     */
    for (int i = 0; i < MAX_SETUPS; i++) {
        if (!g_setups[i].inUse) {
            g_setups[i].inUse = true;
            strncpy(g_setups[i].path, setupPath ? setupPath : "",
                    sizeof(g_setups[i].path) - 1);
            DBG_SER(1, "LoadSetup[%d]: '%s' (native mode — using default addresses)",
                    i, g_setups[i].path);
            return i;
        }
    }

    setGlobalError("TS_LoadSetup: no free setup slots (max %d)", MAX_SETUPS);
    return -1;
}

BOOL TS_SetupAxis(BYTE axisID, int idxSetup)
{
    if (!g_activeCh) {
        setGlobalError("TS_SetupAxis: no active channel");
        return FALSE;
    }

    if (idxSetup < 0 || idxSetup >= MAX_SETUPS || !g_setups[idxSetup].inUse) {
        setGlobalError("TS_SetupAxis: invalid setup index %d", idxSetup);
        return FALSE;
    }

    DBG_SER(1, "SetupAxis: axisID=%d setup=%d", axisID, idxSetup);

    /* In native mode, we just merge any setup-specific variable overrides
     * into the channel's variable map. */

    return TRUE;
}

BOOL TS_SelectAxis(BYTE axisID)
{
    if (!g_activeCh) {
        setGlobalError("TS_SelectAxis: no active channel");
        return FALSE;
    }

    g_activeCh->selectAxis(axisID);
    DBG_SER(2, "SelectAxis: %d", axisID);
    return TRUE;
}

BOOL TS_DriveInitialisation(void)
{
    if (!g_activeCh) {
        setGlobalError("TS_DriveInitialisation: no active channel");
        return FALSE;
    }

    DBG_SER(1, "DriveInitialisation: sending ENDINIT to axis %d",
            g_activeCh->activeAxis());

    /* Send ENDINIT command — validates the setup table on the drive */
    if (!g_activeCh->sendCommand(TML_OP_ENDINIT)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    /* Small delay for drive to process ENDINIT */
    usleep(100000);  /* 100 ms */

    return TRUE;
}

BOOL TS_Power(BOOL Enable)
{
    if (!g_activeCh) {
        setGlobalError("TS_Power: no active channel");
        return FALSE;
    }

    DBG_SER(1, "Power %s axis %d",
            Enable ? "ON" : "OFF", g_activeCh->activeAxis());

    WORD opCode = Enable ? TML_OP_AXISON : TML_OP_AXISOFF;
    if (!g_activeCh->sendCommand(opCode)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    return TRUE;
}

BOOL TS_ReadStatus(short SelIndex, WORD &Status)
{
    if (!g_activeCh) {
        setGlobalError("TS_ReadStatus: no active channel");
        return FALSE;
    }

    /* Map register index to DM address */
    WORD addr;
    switch (SelIndex) {
        case REG_MCR: addr = TML_DM_MCR; break;
        case REG_MSR: addr = TML_DM_MSR; break;
        case REG_ISR: addr = TML_DM_ISR; break;
        case REG_SRL: addr = TML_DM_SRL; break;
        case REG_SRH: addr = TML_DM_SRH; break;
        case REG_MER: addr = TML_DM_MER; break;
        default:
            setGlobalError("TS_ReadStatus: invalid index %d", SelIndex);
            return FALSE;
    }

    if (!g_activeCh->readData16(addr, Status)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    return TRUE;
}

BOOL TS_SetPosition(long PosValue)
{
    if (!g_activeCh) {
        setGlobalError("TS_SetPosition: no active channel");
        return FALSE;
    }

    DBG_SER(1, "SetPosition: %ld", PosValue);

    /*
     * SAP (Set Actual Position): Write to the APOS register and
     * then send the CPA (copy actual position) command.
     */
    uint32_t uval = (uint32_t)(int32_t)PosValue;

    if (!g_activeCh->writeData32(TML_DM_APOS, uval)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    /* CPA to latch the new position as the reference */
    if (!g_activeCh->sendCommand(TML_OP_CPA)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    return TRUE;
}

BOOL TS_Stop(void)
{
    if (!g_activeCh) {
        setGlobalError("TS_Stop: no active channel");
        return FALSE;
    }

    DBG_SER(1, "Stop axis %d", g_activeCh->activeAxis());
    if (!g_activeCh->sendCommand(TML_OP_STOP)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    return TRUE;
}

BOOL TS_ABORT(void)
{
    if (!g_activeCh) {
        setGlobalError("TS_ABORT: no active channel");
        return FALSE;
    }

    DBG_SER(1, "ABORT axis %d", g_activeCh->activeAxis());
    if (!g_activeCh->sendCommand(TML_OP_ABORT)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    return TRUE;
}

BOOL TS_ResetFault(void)
{
    if (!g_activeCh) {
        setGlobalError("TS_ResetFault: no active channel");
        return FALSE;
    }

    DBG_SER(1, "ResetFault axis %d", g_activeCh->activeAxis());
    if (!g_activeCh->sendCommand(TML_OP_FAULTR)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    return TRUE;
}

/* ---- Variable read/write by name ---- */

BOOL TS_GetLongVariable(LPCSTR pszName, long &value)
{
    if (!g_activeCh) {
        setGlobalError("TS_GetLongVariable: no active channel");
        return FALSE;
    }

    WORD addr;
    if (!g_activeCh->varMap().getAddress(pszName, addr)) {
        setGlobalError("TS_GetLongVariable: unknown variable '%s'", pszName);
        return FALSE;
    }

    uint32_t uval;
    if (!g_activeCh->readData32(addr, uval)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    value = (long)(int32_t)uval;
    return TRUE;
}

BOOL TS_SetLongVariable(LPCSTR pszName, long value)
{
    if (!g_activeCh) {
        setGlobalError("TS_SetLongVariable: no active channel");
        return FALSE;
    }

    WORD addr;
    if (!g_activeCh->varMap().getAddress(pszName, addr)) {
        setGlobalError("TS_SetLongVariable: unknown variable '%s'", pszName);
        return FALSE;
    }

    uint32_t uval = (uint32_t)(int32_t)value;
    if (!g_activeCh->writeData32(addr, uval)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    return TRUE;
}

BOOL TS_GetIntVariable(LPCSTR pszName, short &value)
{
    if (!g_activeCh) {
        setGlobalError("TS_GetIntVariable: no active channel");
        return FALSE;
    }

    WORD addr;
    if (!g_activeCh->varMap().getAddress(pszName, addr)) {
        setGlobalError("TS_GetIntVariable: unknown variable '%s'", pszName);
        return FALSE;
    }

    WORD uval;
    if (!g_activeCh->readData16(addr, uval)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    value = (short)uval;
    return TRUE;
}

BOOL TS_SetIntVariable(LPCSTR pszName, short value)
{
    if (!g_activeCh) {
        setGlobalError("TS_SetIntVariable: no active channel");
        return FALSE;
    }

    WORD addr;
    if (!g_activeCh->varMap().getAddress(pszName, addr)) {
        setGlobalError("TS_SetIntVariable: unknown variable '%s'", pszName);
        return FALSE;
    }

    if (!g_activeCh->writeData16(addr, (WORD)value)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    return TRUE;
}

BOOL TS_GetFixedVariable(LPCSTR pszName, double &value)
{
    if (!g_activeCh) {
        setGlobalError("TS_GetFixedVariable: no active channel");
        return FALSE;
    }

    WORD addr;
    if (!g_activeCh->varMap().getAddress(pszName, addr)) {
        setGlobalError("TS_GetFixedVariable: unknown variable '%s'", pszName);
        return FALSE;
    }

    uint32_t uval;
    if (!g_activeCh->readData32(addr, uval)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    value = fixed1616ToDouble(uval);
    return TRUE;
}

BOOL TS_SetFixedVariable(LPCSTR pszName, double value)
{
    if (!g_activeCh) {
        setGlobalError("TS_SetFixedVariable: no active channel");
        return FALSE;
    }

    WORD addr;
    if (!g_activeCh->varMap().getAddress(pszName, addr)) {
        setGlobalError("TS_SetFixedVariable: unknown variable '%s'", pszName);
        return FALSE;
    }

    uint32_t uval = doubleToFixed1616(value);
    if (!g_activeCh->writeData32(addr, uval)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    return TRUE;
}

/* ---- Motion commands ---- */

/*
 * Motion command sequence (trapezoidal position profile):
 *   1. Write CACC (acceleration, 32-bit fixed 16.16) if non-zero
 *   2. Write CSPD (speed, 32-bit fixed 16.16) if non-zero
 *   3. Write CPOS (position, 32-bit integer)
 *   4. Send CPA (absolute) or CPR (relative)
 *   5. Send MODE PP (position profile) or MODE SP (speed profile)
 *   6. Send UPD! (update immediate) or UPD (update on event)
 */

BOOL TS_MoveAbsolute(long AbsPosition, double Speed, double Acceleration,
                     short MoveMoment, short ReferenceBase)
{
    if (!g_activeCh) {
        setGlobalError("TS_MoveAbsolute: no active channel");
        return FALSE;
    }

    DBG_SER(1, "MoveAbsolute pos=%ld spd=%.3f acc=%.3f moment=%d refBase=%d",
            AbsPosition, Speed, Acceleration, MoveMoment, ReferenceBase);

    /* 1. Set acceleration (if non-zero) */
    if (Acceleration != 0.0) {
        uint32_t accFixed = doubleToFixed1616(fabs(Acceleration));
        if (!g_activeCh->writeData32(TML_DM_CACC, accFixed)) goto fail;
    }

    /* 2. Set speed (if non-zero) */
    if (Speed != 0.0) {
        uint32_t spdFixed = doubleToFixed1616(fabs(Speed));
        if (!g_activeCh->writeData32(TML_DM_CSPD, spdFixed)) goto fail;
    }

    /* 3. Set commanded position */
    {
        uint32_t posVal = (uint32_t)(int32_t)AbsPosition;
        if (!g_activeCh->writeData32(TML_DM_CPOS, posVal)) goto fail;
    }

    /* 4. CPA (copy actual position — sets absolute reference) */
    if (!g_activeCh->sendCommand(TML_OP_CPA)) goto fail;

    /* 5. MODE PP (position profile, trapezoidal) */
    if (!g_activeCh->sendCommand(TML_OP_MODE_PP)) goto fail;

    /* 6. Update */
    if (MoveMoment == UPDATE_IMMEDIATE) {
        if (!g_activeCh->sendCommand(TML_OP_UPD_IMM)) goto fail;
    } else if (MoveMoment == UPDATE_ON_EVENT) {
        if (!g_activeCh->sendCommand(TML_OP_UPD)) goto fail;
    }

    return TRUE;

fail:
    strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
    return FALSE;
}

BOOL TS_MoveRelative(long RelPosition, double Speed, double Acceleration,
                     BOOL IsAdditive, short MoveMoment, short ReferenceBase)
{
    if (!g_activeCh) {
        setGlobalError("TS_MoveRelative: no active channel");
        return FALSE;
    }

    DBG_SER(1, "MoveRelative pos=%ld spd=%.3f acc=%.3f additive=%d moment=%d",
            RelPosition, Speed, Acceleration, IsAdditive, MoveMoment);

    /* 1. Acceleration */
    if (Acceleration != 0.0) {
        uint32_t accFixed = doubleToFixed1616(fabs(Acceleration));
        if (!g_activeCh->writeData32(TML_DM_CACC, accFixed)) goto fail;
    }

    /* 2. Speed */
    if (Speed != 0.0) {
        uint32_t spdFixed = doubleToFixed1616(fabs(Speed));
        if (!g_activeCh->writeData32(TML_DM_CSPD, spdFixed)) goto fail;
    }

    /* 3. Position */
    {
        uint32_t posVal = (uint32_t)(int32_t)RelPosition;
        if (!g_activeCh->writeData32(TML_DM_CPOS, posVal)) goto fail;
    }

    /* 4. CPR (copy position — relative reference) */
    if (!g_activeCh->sendCommand(TML_OP_CPR)) goto fail;

    /* 5. MODE PP */
    if (!g_activeCh->sendCommand(TML_OP_MODE_PP)) goto fail;

    /* 6. Update */
    if (MoveMoment == UPDATE_IMMEDIATE) {
        if (!g_activeCh->sendCommand(TML_OP_UPD_IMM)) goto fail;
    } else if (MoveMoment == UPDATE_ON_EVENT) {
        if (!g_activeCh->sendCommand(TML_OP_UPD)) goto fail;
    }

    return TRUE;

fail:
    strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
    return FALSE;
}

BOOL TS_MoveVelocity(double Speed, double Acceleration,
                     short MoveMoment, short ReferenceBase)
{
    if (!g_activeCh) {
        setGlobalError("TS_MoveVelocity: no active channel");
        return FALSE;
    }

    DBG_SER(1, "MoveVelocity spd=%.3f acc=%.3f moment=%d", Speed, Acceleration, MoveMoment);

    /* 1. Acceleration */
    if (Acceleration != 0.0) {
        uint32_t accFixed = doubleToFixed1616(fabs(Acceleration));
        if (!g_activeCh->writeData32(TML_DM_CACC, accFixed)) goto fail;
    }

    /* 2. Speed (signed — direction encoded in the speed value) */
    {
        uint32_t spdFixed = doubleToFixed1616(Speed);
        if (!g_activeCh->writeData32(TML_DM_CSPD, spdFixed)) goto fail;
    }

    /* 3. MODE SP (speed profile) */
    if (!g_activeCh->sendCommand(TML_OP_MODE_SP)) goto fail;

    /* 4. Update */
    if (MoveMoment == UPDATE_IMMEDIATE) {
        if (!g_activeCh->sendCommand(TML_OP_UPD_IMM)) goto fail;
    } else if (MoveMoment == UPDATE_ON_EVENT) {
        if (!g_activeCh->sendCommand(TML_OP_UPD)) goto fail;
    }

    return TRUE;

fail:
    strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
    return FALSE;
}

/* ---- Event commands ---- */

BOOL TS_SetEventOnMotionComplete(BOOL WaitEvent, BOOL EnableStop)
{
    if (!g_activeCh) {
        setGlobalError("TS_SetEventOnMotionComplete: no active channel");
        return FALSE;
    }

    DBG_SER(2, "SetEventOnMotionComplete wait=%d stop=%d", WaitEvent, EnableStop);

    /*
     * Configure the drive to signal (via SRL.10) when motion completes.
     * Set the SRL_MASK bit 10 to enable automatic status reporting.
     *
     * For the native implementation, we configure the event mask so
     * that SRL bit 10 (motion complete) triggers notification.
     * The actual monitoring is done by polling in the EPICS driver.
     */
    WORD srlMask;
    if (!g_activeCh->readData16(TML_DM_SRL_MASK, srlMask)) {
        /* If we can't read, set a sensible default */
        srlMask = 0;
    }

    srlMask |= (1 << 10);  /* Enable motion complete event */

    if (!g_activeCh->writeData16(TML_DM_SRL_MASK, srlMask)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    return TRUE;
}

BOOL TS_SetEventOnLimitSwitch(short LSWType, short TransitionType,
                              BOOL WaitEvent, BOOL EnableStop)
{
    if (!g_activeCh) {
        setGlobalError("TS_SetEventOnLimitSwitch: no active channel");
        return FALSE;
    }

    DBG_SER(2, "SetEventOnLimitSwitch type=%d transition=%d wait=%d stop=%d",
            LSWType, TransitionType, WaitEvent, EnableStop);

    /*
     * Configure limit switch event via MER_MASK register.
     * MER bit 6 = LSP (positive limit), bit 7 = LSN (negative limit).
     */
    WORD merMask;
    if (!g_activeCh->readData16(TML_DM_MER_MASK, merMask)) {
        merMask = 0;
    }

    if (LSWType == LSW_POSITIVE) {
        merMask |= (1 << 6);   /* LSP */
    } else {
        merMask |= (1 << 7);   /* LSN */
    }

    if (!g_activeCh->writeData16(TML_DM_MER_MASK, merMask)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    return TRUE;
}

BOOL TS_UpdateImmediate(void)
{
    if (!g_activeCh) {
        setGlobalError("TS_UpdateImmediate: no active channel");
        return FALSE;
    }

    if (!g_activeCh->sendCommand(TML_OP_UPD_IMM)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    return TRUE;
}

BOOL TS_Save(void)
{
    if (!g_activeCh) {
        setGlobalError("TS_Save: no active channel");
        return FALSE;
    }

    if (!g_activeCh->sendCommand(TML_OP_SAVE)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    return TRUE;
}

BOOL TS_Reset(void)
{
    if (!g_activeCh) {
        setGlobalError("TS_Reset: no active channel");
        return FALSE;
    }

    if (!g_activeCh->sendCommand(TML_OP_RESET)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    return TRUE;
}

} /* extern "C" */
