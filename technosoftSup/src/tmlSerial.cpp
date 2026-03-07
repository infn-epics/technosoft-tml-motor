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
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>

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
    map_["SCR"]   = TML_DM_SCR;

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

void TmlVariableMap::merge(const TmlVariableMap &other)
{
    for (auto &kv : other.map_) {
        map_[kv.first] = kv.second;
    }
}

int TmlVariableMap::loadFromZip(const char *zipPath)
{
    /*
     * Extract 'variables.cfg' from the .t.zip file using unzip -p
     * (pipe to stdout).  Each line has the format:
     *   TYPE  NAME  @0xADDR [optional flags]
     * e.g.
     *   LONG    TPOS    @0x02B2
     *   UINT    MER     @0x08FC
     *   FIXED   CSPD    @0x02A0
     */
    /* Verify the file exists before attempting to unzip; missing setup files
     * are a common misconfiguration and should be flagged clearly. */
    if (access(zipPath, R_OK) != 0) {
        DBG_SER(0, "loadFromZip: setup file not found or not readable: '%s' (%s)",
                zipPath, strerror(errno));
        return 0;
    }

    char cmd[1024];
    snprintf(cmd, sizeof(cmd), "unzip -p '%s' variables.cfg 2>/dev/null", zipPath);

    FILE *fp = popen(cmd, "r");
    if (!fp) {
        DBG_SER(0, "loadFromZip: popen failed for '%s'", zipPath);
        return 0;
    }

    int count = 0;
    char line[512];
    while (fgets(line, sizeof(line), fp)) {
        /* Skip comments and empty lines */
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
            continue;

        /* Parse: TYPE  NAME  @0xADDR */
        char type[32] = {0}, name[64] = {0}, addrStr[32] = {0};
        if (sscanf(line, "%31s %63s %31s", type, name, addrStr) < 3)
            continue;

        /* addrStr should start with '@0x' */
        if (addrStr[0] != '@' || addrStr[1] != '0' ||
            (addrStr[2] != 'x' && addrStr[2] != 'X'))
            continue;

        unsigned int addr = 0;
        if (sscanf(addrStr + 1, "%x", &addr) != 1)
            continue;

        map_[name] = (WORD)addr;
        count++;
    }

    pclose(fp);
    DBG_SER(1, "loadFromZip: loaded %d variable addresses from '%s'", count, zipPath);
    return count;
}

/* ================================================================= */
/*               TmlChannel — construction / destruction             */
/* ================================================================= */

TmlChannel::TmlChannel()
    : fd_(-1)
    , hostId_(1)
    , activeAxisId_(255)
    , channelType_(CHANNEL_RS232)
    , isTcp_(false)
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

    /* TML serial: 8 data bits, 1 stop bit, no parity (8N1) */
    tty.c_cflag &= ~PARENB;        /* No parity */
    tty.c_cflag &= ~CSTOPB;        /* 1 stop bit */
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
    isTcp_       = false;

    /* ---- Detect TCP/IP mode ---- */
    if (channelType == CHANNEL_TCP || channelType == CHANNEL_XPORT_IP
        || looksLikeTcp(devPath)) {
        printf("tmlSerial: connecting TCP to '%s' (hostId=%d)\n",
               devPath, hostId_);
        fd_ = connectTcp(devPath);
        if (fd_ < 0) {
            printf("tmlSerial: TCP connection FAILED — %s\n", lastError_);
            return -1;
        }
        isTcp_ = true;
        channelType_ = CHANNEL_TCP;
        printf("tmlSerial: TCP connected '%s' fd=%d\n", devPath, fd_);

        /* Give the serial bridge (Moxa NPort, XPORT, etc.) time to
         * initialise after TCP connect, then drain any stale bytes. */
        usleep(200000);  /* 200 ms */
        uint8_t junk[64];
        int stale = readBytes(junk, sizeof(junk), 50);
        if (stale > 0) {
            DBG_SER(1, "Drained %d stale bytes after connect", stale);
            hexDump("DRAIN", junk, stale);
        }

        DBG_SER(1, "Opened TCP '%s' fd=%d hostId=%d",
                devPath, fd_, hostId_);
        return fd_;
    }

    /* ---- Serial port mode ---- */
    printf("tmlSerial: opening serial '%s' (baud=%u, 8N1, %s, hostId=%d)\n",
           devPath, (unsigned)baudRate,
           channelType == CHANNEL_RS485 ? "RS-485" : "RS-232",
           hostId_);

    fd_ = ::open(devPath, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        setError("Cannot open '%s': %s", devPath, strerror(errno));
        printf("tmlSerial: serial open FAILED — %s\n", lastError_);
        return -1;
    }

    /* Clear O_NONBLOCK after open */
    int flags = fcntl(fd_, F_GETFL, 0);
    if (flags >= 0)
        fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    if (!configurePort(baudRate)) {
        printf("tmlSerial: serial configure FAILED — %s\n", lastError_);
        ::close(fd_);
        fd_ = -1;
        return -1;
    }

    printf("tmlSerial: serial opened '%s' fd=%d\n", devPath, fd_);
    DBG_SER(1, "Opened serial '%s' fd=%d hostId=%d baud=%u type=%d",
            devPath, fd_, hostId_, (unsigned)baudRate, channelType_);

    return fd_;
}

/* ================================================================= */
/*               TCP/IP support (XPORT, ser2net, etc.)               */
/* ================================================================= */

bool TmlChannel::looksLikeTcp(const char *devPath)
{
    /* Heuristic: contains ':' and does NOT start with '/' (device path) */
    if (!devPath || devPath[0] == '/' || devPath[0] == '.')
        return false;
    return (strchr(devPath, ':') != nullptr);
}

int TmlChannel::connectTcp(const char *hostPort)
{
    /* Parse "host:port" */
    char hostBuf[256];
    strncpy(hostBuf, hostPort, sizeof(hostBuf) - 1);
    hostBuf[sizeof(hostBuf) - 1] = '\0';

    char *colon = strrchr(hostBuf, ':');
    if (!colon) {
        setError("TCP address must be 'host:port', got '%s'", hostPort);
        return -1;
    }
    *colon = '\0';
    const char *host = hostBuf;
    int port = atoi(colon + 1);
    if (port <= 0 || port > 65535) {
        setError("Invalid TCP port in '%s'", hostPort);
        return -1;
    }

    DBG_SER(1, "TCP connecting to %s:%d", host, port);

    /* Resolve hostname */
    struct addrinfo hints, *res = nullptr;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    char portStr[8];
    snprintf(portStr, sizeof(portStr), "%d", port);

    int gai = getaddrinfo(host, portStr, &hints, &res);
    if (gai != 0) {
        setError("getaddrinfo('%s'): %s", host, gai_strerror(gai));
        return -1;
    }

    /* Try each resolved address */
    int sock = -1;
    for (struct addrinfo *rp = res; rp; rp = rp->ai_next) {
        sock = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (sock < 0) continue;

        /* Set a 5-second connect timeout via non-blocking + select */
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);

        int cret = ::connect(sock, rp->ai_addr, rp->ai_addrlen);
        if (cret < 0 && errno == EINPROGRESS) {
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(sock, &wfds);
            struct timeval tv = {5, 0};
            if (select(sock + 1, nullptr, &wfds, nullptr, &tv) > 0) {
                int err = 0;
                socklen_t len = sizeof(err);
                getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &len);
                if (err == 0) cret = 0;
            }
        }

        if (cret == 0) {
            /* Connected! Restore blocking mode */
            fcntl(sock, F_SETFL, flags & ~O_NONBLOCK);

            /* Disable Nagle for low-latency frame exchange */
            int one = 1;
            setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

            DBG_SER(1, "TCP connected to %s:%d fd=%d", host, port, sock);
            break;
        }

        ::close(sock);
        sock = -1;
    }

    freeaddrinfo(res);

    if (sock < 0) {
        setError("TCP connect to '%s' failed: %s", hostPort, strerror(errno));
    }
    return sock;
}

void TmlChannel::close()
{
    if (fd_ >= 0) {
        DBG_SER(1, "Closing fd=%d%s", fd_, isTcp_ ? " (TCP)" : "");
        ::close(fd_);
        fd_ = -1;
        isTcp_ = false;
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

    /* Ensure bytes are drained to the hardware (serial only) */
    if (!isTcp_)
        tcdrain(fd_);
    return true;
}

/* ================================================================= */
/*               Hex-dump trace helper                               */
/* ================================================================= */

void TmlChannel::hexDump(const char *tag, const uint8_t *buf, int len)
{
    if (drvTmlDebug < 4 || len <= 0) return;

    /* Format: "TX 08 00F0 B004 0FF1 0228  [ck=xx]" */
    char line[256];
    int pos = snprintf(line, sizeof(line), "%-3s", tag);

    for (int i = 0; i < len && pos < (int)sizeof(line) - 4; i++) {
        pos += snprintf(line + pos, sizeof(line) - pos, " %02X", buf[i]);
    }

    /* Also decode structure if it's a valid frame */
    if (len >= 6) {
        int payloadLen = buf[0];
        if (payloadLen >= 4 && 1 + payloadLen + 1 <= len) {
            WORD addr   = ((WORD)buf[1] << 8) | buf[2];
            WORD opCode = ((WORD)buf[3] << 8) | buf[4];
            int nData   = (payloadLen - 4) / 2;
            pos += snprintf(line + pos, sizeof(line) - pos,
                            "  | addr=0x%04X op=0x%04X nData=%d",
                            addr, opCode, nData);
            for (int i = 0; i < nData && i < 4; i++) {
                int off = 5 + 2 * i;
                WORD w = ((WORD)buf[off] << 8) | buf[off + 1];
                pos += snprintf(line + pos, sizeof(line) - pos,
                                " d[%d]=0x%04X", i, w);
            }
        }
    }

    printf("tmlSerial TRACE %s\n", line);
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
/*               ACK handling                                        */
/* ================================================================= */

bool TmlChannel::waitAck(int timeoutMs)
{
    /*
     * After sending a command the drive replies with a single ACK byte
     * (0x4F = 'O').  On the wire there may be:
     *   - 0x0D / 0x0A padding (CR/LF inserted by bridges)
     *   - 0x00 null bytes (Moxa NPort, socat, bridge init artefacts)
     *   - stale response bytes from a previous operation that wasn't
     *     fully consumed (e.g. after a timeout/retry on another axis)
     *
     * Strategy: keep reading and discarding ALL unexpected bytes until
     * the ACK byte arrives or the timeout expires.  This is safe because
     * after a command the only valid single-byte response from the drive
     * is 0x4F.  Any other byte is junk/stale data.
     */
    uint8_t byte;
    int elapsed = 0;
    const int stepMs = 2;  /* small read timeout per attempt */
    int junkCount = 0;
    while (elapsed < timeoutMs) {
        int n = readBytes(&byte, 1, stepMs);
        if (n != 1) {
            elapsed += stepMs;
            continue;
        }
        if (byte == TML_ACK_BYTE) {
            if (junkCount > 0)
                DBG_SER(1, "ACK received after skipping %d unexpected bytes", junkCount);
            else
                DBG_SER(3, "ACK received");
            return true;
        }
        /* Skip any non-ACK byte — log it for diagnostics */
        junkCount++;
        DBG_SER(2, "waitAck: skipping byte 0x%02X (%d skipped so far)", byte, junkCount);
    }
    setError("ACK timeout (%d ms, skipped %d bytes)", timeoutMs, junkCount);
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

    /* Drain any stale bytes left in the receive buffer from previous
     * operations (partial responses, async drive notifications, etc.).
     * This prevents stale data from being misinterpreted as the ACK
     * or response to the command we're about to send. */
    {
        uint8_t junk[64];
        int stale = readBytes(junk, sizeof(junk), 1);  /* 1 ms non-blocking drain */
        if (stale > 0) {
            DBG_SER(1, "Drained %d stale RX bytes before send", stale);
            hexDump("DRAIN", junk, stale);
        }
    }

    DBG_SER(3, "TX [%d bytes] addr=0x%04X op=0x%04X nData=%d",
            len, msg.addr, msg.opCode, msg.nData);
    hexDump("TX", buf, len);

    if (!writeBytes(buf, len))
        return false;

    /* Wait for ACK */
    if (!waitAck()) {
        /* ACK failed — retry the command once after a short delay.
         * Do NOT use resync (0x0D byte) — on a multi-drive RS-485 bus
         * the sync byte is received by ALL drives, corrupting bus state. */
        DBG_SER(1, "No ACK, retrying command once after drain+delay");
        usleep(50000);  /* 50 ms settle */

        /* Drain anything that arrived during the wait */
        uint8_t junk[64];
        int stale = readBytes(junk, sizeof(junk), 10);
        if (stale > 0) {
            DBG_SER(1, "Drained %d bytes before retry", stale);
            hexDump("DRAIN", junk, stale);
        }

        if (!writeBytes(buf, len))
            return false;
        if (!waitAck()) {
            /* Still no ACK — give up */
            return false;
        }
    }

    return true;
}

bool TmlChannel::receiveMessage(TmlMsg &msg, int timeoutMs)
{
    /* First read the length byte, skipping any padding/junk bytes.
     * On RS-232 via socat/Moxa, null bytes (0x00), CR (0x0D) and
     * LF (0x0A) can appear between frames.  Valid TML payload lengths
     * are 4..12, so any byte outside that range is junk. */
    uint8_t lenByte;
    int elapsed = 0;
    const int stepMs = 2;
    bool gotLen = false;
    int skipped = 0;
    while (elapsed < timeoutMs) {
        int n = readBytes(&lenByte, 1, stepMs);
        if (n != 1) {
            elapsed += stepMs;
            continue;
        }
        if (lenByte >= 4 && lenByte <= 12) {
            gotLen = true;
            break;
        }
        /* Skip junk byte */
        skipped++;
        DBG_SER(2, "receiveMessage: skipping junk byte 0x%02X (%d skipped)", lenByte, skipped);
    }
    if (!gotLen) {
        setError("Timeout waiting for response length byte (skipped %d junk bytes)", skipped);
        return false;
    }
    if (skipped > 0)
        DBG_SER(1, "receiveMessage: skipped %d junk bytes before length byte 0x%02X", skipped, lenByte);

    int payloadLen = lenByte;

    /* Read the rest: payload + checksum */
    uint8_t buf[TML_MAX_MSG_BYTES];
    buf[0] = lenByte;
    int remaining = payloadLen + 1;  /* payload bytes + checksum byte */

    int n = readBytes(buf + 1, remaining, timeoutMs);
    if (n != remaining) {
        setError("Incomplete response: got %d of %d bytes", n, remaining);
        return false;
    }

    int totalLen = 1 + payloadLen + 1;

    DBG_SER(3, "RX [%d bytes] raw: %02X %02X %02X %02X %02X ...",
            totalLen, buf[0], buf[1], buf[2],
            totalLen > 3 ? buf[3] : 0, totalLen > 4 ? buf[4] : 0);
    hexDump("RX", buf, totalLen);

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

bool TmlChannel::sendMcrConfig(WORD mask, WORD value)
{
    /*
     * CHMOD instruction (TML ISA opcode 0x5909 + 2 data words).
     *
     * This is a TML assembly instruction processed by the drive's
     * instruction processor.  It modifies MCR as:
     *     MCR = (MCR & mask) | value
     * and triggers the associated state machine transitions (mode change,
     * reference base switch, etc.).
     *
     * This is NOT the same as writing MCR via the online communication
     * protocol (WriteData16) — CHMOD has side effects in the motion
     * control firmware that a raw register write does not.
     */
    TmlMsg msg;
    msg.addr   = (WORD)activeAxisId_ << 4;
    msg.opCode = TML_OP_MCR_CONFIG;   /* 0x5909 */
    msg.nData  = 2;
    msg.data[0] = mask;
    msg.data[1] = value;

    DBG_SER(2, "CHMOD mask=0x%04X val=0x%04X → axis %d", mask, value, activeAxisId_);

    return sendMessage(msg);
}

bool TmlChannel::sendSAP(int32_t position)
{
    /*
     * SAP value32: Set Actual Position (TML Manual p.265).
     * Opcode = 0x8400, with 2 data words:
     *   Data[0] = LOWORD(position)
     *   Data[1] = HIWORD(position)
     * Also corrects the reference value so that the difference between
     * the position reference and actual position is preserved.
     */
    TmlMsg msg;
    msg.addr   = (WORD)activeAxisId_ << 4;
    msg.opCode = TML_OP_SAP;
    msg.nData  = 2;
    msg.data[0] = (WORD)((uint32_t)position & 0xFFFF);
    msg.data[1] = (WORD)(((uint32_t)position >> 16) & 0xFFFF);

    DBG_SER(2, "SAP position=%d (0x%08X)", (int)position,
            (unsigned)(uint32_t)position);

    return sendMessage(msg);
}

bool TmlChannel::writeData16(WORD address, WORD value)
{
    /* Online communication protocol: WriteData16 (opcode 0x9004)
     * Data[0] = DM address, Data[1] = value.
     * Verified from TML_lib libtmlcomm.so SendData @ 0xa47c-0xa49e */
    TmlMsg msg;
    msg.addr   = (WORD)activeAxisId_ << 4;
    msg.opCode = TML_OP_WRITE_DATA_16;
    msg.nData  = 2;
    msg.data[0] = address;
    msg.data[1] = value;

    DBG_SER(2, "WriteData16 addr=0x%04X val=0x%04X", address, value);

    return sendMessage(msg);
}

bool TmlChannel::writeData32(WORD address, uint32_t value)
{
    /* Online communication protocol: WriteData32 (opcode 0x9005)
     * Data[0] = DM address, Data[1] = low word, Data[2] = high word.
     * Verified from TML_lib libtmlcomm.so SendData @ 0xa47c-0xa49e */
    TmlMsg msg;
    msg.addr   = (WORD)activeAxisId_ << 4;
    msg.opCode = TML_OP_WRITE_DATA_32;
    msg.nData  = 3;
    msg.data[0] = address;
    msg.data[1] = (WORD)(value & 0xFFFF);         /* Low word */
    msg.data[2] = (WORD)((value >> 16) & 0xFFFF); /* High word */

    DBG_SER(2, "WriteData32 addr=0x%04X val=0x%08X",
            address, (unsigned)value);

    return sendMessage(msg);
}

bool TmlChannel::readData16(WORD address, WORD &value)
{
    /* Build GiveMeData16 request.
     * Wire data layout: data[0] = sender axis ID, data[1] = DM address.
     * (Verified against TML_lib ReceiveData @ libtmlcomm.so:0x9d2d) */
    TmlMsg req;
    req.addr    = (WORD)activeAxisId_ << 4;
    req.opCode  = TML_OP_GIVE_ME_DATA_16;
    req.nData   = 2;
    req.data[0] = ((WORD)hostId_ << 4) | 0x0001;  /* Sender: host with host-bit */
    req.data[1] = address;                         /* DM address to read */

    DBG_SER(2, "ReadData16 addr=0x%04X", address);

    if (!sendMessage(req))
        return false;

    /* Wait for TakeData16 response */
    TmlMsg resp;
    if (!receiveMessage(resp, TML_RESP_TIMEOUT_MS))
        return false;

    /* TakeData16 layout: data[0]=sender(drive), data[1]=addr echo, data[2]=value
     * nData = 3 (verified against TML_lib ReceiveData validation @ 0x9f17) */
    if (resp.opCode != TML_OP_TAKE_DATA_16 || resp.nData < 3) {
        setError("Unexpected response: opCode=0x%04X nData=%d (expected TakeData16 0x%04X nData>=3)",
                 resp.opCode, resp.nData, TML_OP_TAKE_DATA_16);
        return false;
    }

    /* resp.data[0] = drive axis ID, data[1] = address echo, data[2] = value */
    value = resp.data[2];

    DBG_SER(2, "ReadData16 addr=0x%04X → 0x%04X", address, value);
    return true;
}

bool TmlChannel::readData32(WORD address, uint32_t &value)
{
    /* Build GiveMeData32 request.
     * Wire data layout: data[0] = sender axis ID, data[1] = DM address. */
    TmlMsg req;
    req.addr    = (WORD)activeAxisId_ << 4;
    req.opCode  = TML_OP_GIVE_ME_DATA_32;
    req.nData   = 2;
    req.data[0] = ((WORD)hostId_ << 4) | 0x0001;   /* Sender: host with host-bit */
    req.data[1] = address;                          /* DM address to read */

    DBG_SER(2, "ReadData32 addr=0x%04X", address);

    if (!sendMessage(req))
        return false;

    /* Wait for TakeData32 response */
    TmlMsg resp;
    if (!receiveMessage(resp, TML_RESP_TIMEOUT_MS))
        return false;

    /* TakeData32 layout: data[0]=sender(drive), data[1]=addr echo,
     *                    data[2]=low word, data[3]=high word.  nData=4 */
    if (resp.opCode != TML_OP_TAKE_DATA_32 || resp.nData < 4) {
        setError("Unexpected response: opCode=0x%04X nData=%d (expected TakeData32 0x%04X nData>=4)",
                 resp.opCode, resp.nData, TML_OP_TAKE_DATA_32);
        return false;
    }

    /* resp.data[0] = drive axis ID, data[1] = addr echo,
     * data[2] = low word, data[3] = high word */
    value = ((uint32_t)resp.data[3] << 16) | (uint32_t)resp.data[2];

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

    printf("tmlSerial: TS_OpenChannel OK — transport=%s dev='%s' fd=%d slot=%d hostId=%d\n",
           ch->isTcp() ? "TCP" :
               (btType == CHANNEL_RS485 ? "RS-485" : "RS-232"),
           pszDevName, fd, slot, nHostID);
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
    for (int i = 0; i < MAX_SETUPS; i++) {
        if (!g_setups[i].inUse) {
            g_setups[i].inUse = true;
            strncpy(g_setups[i].path, setupPath ? setupPath : "",
                    sizeof(g_setups[i].path) - 1);

            /*
             * Parse the .t.zip setup file for firmware-specific variable
             * addresses.  The variables.cfg inside the ZIP maps TML variable
             * names to DM addresses that may differ from the "standard"
             * TML manual addresses depending on the firmware version.
             */
            int nVars = g_setups[i].varMap.loadFromZip(g_setups[i].path);
            if (nVars > 0) {
                DBG_SER(1, "LoadSetup[%d]: '%s' — loaded %d firmware variable addresses",
                        i, g_setups[i].path, nVars);
            } else {
                /* Level-0: always visible — missing/empty setup files cause
                 * 'unknown variable' errors that are hard to diagnose otherwise. */
                DBG_SER(0, "LoadSetup[%d]: WARNING — '%s' yielded no variable addresses; "
                        "only built-in defaults will be used (SCR, MER, etc.)",
                        i, g_setups[i].path);
            }
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

    /* Merge setup-specific variable addresses into the channel's map.
     * This overrides the default TML manual addresses with the correct
     * firmware-specific addresses from the .t.zip setup file. */
    g_activeCh->varMap().merge(g_setups[idxSetup].varMap);
    DBG_SER(1, "SetupAxis: merged %zu variable addresses from setup into channel",
            g_setups[idxSetup].varMap.size());

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

    /* Send ENDINIT command — validates the setup table on the drive.
     * Retry up to 3 times because the drive/XPORT may not respond
     * immediately after connection (especially over TCP). */
    const int maxRetries = 3;
    for (int i = 0; i < maxRetries; i++) {
        if (g_activeCh->sendCommand(TML_OP_ENDINIT)) {
            /* Small delay for drive to process ENDINIT */
            usleep(100000);  /* 100 ms */
            return TRUE;
        }
        DBG_SER(1, "DriveInitialisation: ENDINIT attempt %d/%d failed: %s",
                i + 1, maxRetries, g_activeCh->lastError());
        if (i + 1 < maxRetries)
            usleep(500000);  /* 500 ms before retry */
    }

    strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
    return FALSE;
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

    /* Map register index to variable name, then resolve via varMap.
     * This uses the firmware-specific addresses from the .t.zip setup file
     * rather than hardcoded TML manual addresses. */
    const char *varName = nullptr;
    switch (SelIndex) {
        case REG_MCR: varName = "MCR"; break;
        case REG_MSR: varName = "MSR"; break;
        case REG_ISR: varName = "ISR"; break;
        case REG_SRL: varName = "SRL"; break;
        case REG_SRH: varName = "SRH"; break;
        case REG_MER: varName = "MER"; break;
        default:
            setGlobalError("TS_ReadStatus: invalid index %d", SelIndex);
            return FALSE;
    }

    WORD addr;
    if (!g_activeCh->varMap().getAddress(varName, addr)) {
        setGlobalError("TS_ReadStatus: variable '%s' not in map", varName);
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
     * Use the SAP (Set Actual Position) instruction.
     * SAP sets APOS and also corrects the reference value so that
     * the difference (reference − APOS) is preserved.
     * (TML Manual p.265, opcode 0x8400 + 2 data words)
     */
    if (!g_activeCh->sendSAP((int32_t)PosValue)) {
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
    if (!g_activeCh->sendCommand(TML_OP_STOP_IMM)) {
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

    DBG_SER(1, "ResetFault axis %d — clearing MER", g_activeCh->activeAxis());

    /* Read current MER for diagnostics */
    WORD mer = 0;
    WORD merAddr = TML_DM_MER;
    {
        WORD resolved;
        if (g_activeCh->varMap().getAddress("MER", resolved))
            merAddr = resolved;
    }
    g_activeCh->readData16(merAddr, mer);
    if (mer != 0)
        DBG_SER(1, "ResetFault: MER=0x%04X before clear", mer);

    /* Clear faults by writing 0 to MER (Motion Error Register).
     * This is safer than sending RESET/0x0402 which would reset the DSP. */
    if (!g_activeCh->writeData16(merAddr, 0)) {
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
 * Helper: resolve a variable name to a DM address via the active channel's varMap.
 * Falls back to the compiled default if the variable is not in the map.
 */
static WORD resolveAddr(const char *name, WORD fallback)
{
    if (!g_activeCh) return fallback;
    WORD addr;
    if (g_activeCh->varMap().getAddress(name, addr))
        return addr;
    return fallback;
}

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
        if (!g_activeCh->writeData32(resolveAddr("CACC", TML_DM_CACC), accFixed)) goto fail;
    }

    /* 2. Set speed (if non-zero) */
    if (Speed != 0.0) {
        uint32_t spdFixed = doubleToFixed1616(fabs(Speed));
        if (!g_activeCh->writeData32(resolveAddr("CSPD", TML_DM_CSPD), spdFixed)) goto fail;
    }

    /* 3. Set commanded position */
    {
        uint32_t posVal = (uint32_t)(int32_t)AbsPosition;
        if (!g_activeCh->writeData32(resolveAddr("CPOS", TML_DM_CPOS), posVal)) goto fail;
    }

    /* 4. CPA: Set absolute reference mode (MCR bit 13 = 1) */
    if (!g_activeCh->sendMcrConfig(TML_MCR_CPA_MASK, TML_MCR_CPA_VAL)) goto fail;

    /* 5. MODE PP3 (position profile, all loops: position+speed+current) */
    if (!g_activeCh->sendMcrConfig(TML_MCR_MODE_PP3_MASK, TML_MCR_MODE_PP3_VAL)) goto fail;

    /* 5b. FROM_REFERENCE: set MCR bit 5 if requested (trajectory starts
     *     from reference position rather than measured position) */
    if (ReferenceBase == FROM_REFERENCE) {
        if (!g_activeCh->sendMcrConfig(0xFFFF, MCR_BIT_REF_BASE)) goto fail;
    }

    /* 6. Update */
    if (MoveMoment == UPDATE_IMMEDIATE) {
        if (!g_activeCh->sendCommand(TML_OP_UPD_IMM)) goto fail;
    } else if (MoveMoment == UPDATE_ON_EVENT) {
        if (!g_activeCh->sendCommand(TML_OP_UPD)) goto fail;
    }

    /* Post-UPD diagnostic: check if motion actually started */
    {
        WORD srl = 0, mer = 0;
        g_activeCh->readData16(resolveAddr("SRL", TML_DM_SRL), srl);
        g_activeCh->readData16(resolveAddr("MER", TML_DM_MER), mer);
        bool mc = (srl & (1 << 10)) != 0;  /* motion complete */
        DBG_SER(1, "MoveAbsolute POST-UPD: SRL=0x%04X MER=0x%04X motionComplete=%d",
                srl, mer, mc);
        if (mc && mer != 0)
            DBG_SER(0, "WARNING: motion blocked — MER=0x%04X (limit switches active?)", mer);
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
        if (!g_activeCh->writeData32(resolveAddr("CACC", TML_DM_CACC), accFixed)) goto fail;
    }

    /* 2. Speed */
    if (Speed != 0.0) {
        uint32_t spdFixed = doubleToFixed1616(fabs(Speed));
        if (!g_activeCh->writeData32(resolveAddr("CSPD", TML_DM_CSPD), spdFixed)) goto fail;
    }

    /* 3. Position */
    {
        uint32_t posVal = (uint32_t)(int32_t)RelPosition;
        if (!g_activeCh->writeData32(resolveAddr("CPOS", TML_DM_CPOS), posVal)) goto fail;
    }

    /* 4. CPR: Set relative reference mode (MCR bit 13 = 0) */
    if (!g_activeCh->sendMcrConfig(TML_MCR_CPR_MASK, TML_MCR_CPR_VAL)) goto fail;

    /* 5. MODE PP3 (position profile, all loops: position+speed+current) */
    if (!g_activeCh->sendMcrConfig(TML_MCR_MODE_PP3_MASK, TML_MCR_MODE_PP3_VAL)) goto fail;

    /* 5b. FROM_REFERENCE: set MCR bit 5 if requested */
    if (ReferenceBase == FROM_REFERENCE) {
        if (!g_activeCh->sendMcrConfig(0xFFFF, MCR_BIT_REF_BASE)) goto fail;
    }

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
        if (!g_activeCh->writeData32(resolveAddr("CACC", TML_DM_CACC), accFixed)) goto fail;
    }

    /* 2. Speed (signed — direction encoded in the speed value) */
    {
        uint32_t spdFixed = doubleToFixed1616(Speed);
        if (!g_activeCh->writeData32(resolveAddr("CSPD", TML_DM_CSPD), spdFixed)) goto fail;
    }

    /* 3. MODE SP1 (speed profile with current loop) */
    if (!g_activeCh->sendMcrConfig(TML_MCR_MODE_SP1_MASK, TML_MCR_MODE_SP1_VAL)) goto fail;

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
    WORD srlMaskAddr = resolveAddr("SRL_MASK", TML_DM_SRL_MASK);
    WORD srlMask;
    if (!g_activeCh->readData16(srlMaskAddr, srlMask)) {
        /* If we can't read, set a sensible default */
        srlMask = 0;
    }

    srlMask |= (1 << 10);  /* Enable motion complete event */

    if (!g_activeCh->writeData16(srlMaskAddr, srlMask)) {
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
    WORD merMaskAddr = resolveAddr("MER_MASK", TML_DM_MER_MASK);
    WORD merMask;
    if (!g_activeCh->readData16(merMaskAddr, merMask)) {
        merMask = 0;
    }

    if (LSWType == LSW_POSITIVE) {
        merMask |= (1 << 6);   /* LSP */
    } else {
        merMask |= (1 << 7);   /* LSN */
    }

    if (!g_activeCh->writeData16(merMaskAddr, merMask)) {
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

    /*
     * Note: There is no SAVE/EEPROM-write instruction in the TML ISA.
     * The TML_lib TS_Save function uses proprietary mechanisms.
     * For now, this is a no-op that logs a warning.
     */
    DBG_SER(0, "TS_Save: EEPROM save not implemented in native protocol (no TML ISA equivalent)");

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

BOOL TS_ClearLimitSwitchEvent(void)
{
    if (!g_activeCh) {
        setGlobalError("TS_ClearLimitSwitchEvent: no active channel");
        return FALSE;
    }

    /* Remove limit switch event bits from MER_MASK so the drive no longer
     * latches LSP/LSN in MER.  Then clear any already-latched LS bits
     * in MER itself.  After this, MER bits 6-7 reflect live input state. */
    WORD merMaskAddr = resolveAddr("MER_MASK", TML_DM_MER_MASK);
    WORD merMask = 0;
    g_activeCh->readData16(merMaskAddr, merMask);
    WORD orig = merMask;
    merMask &= ~(WORD)((1 << 6) | (1 << 7));
    if (merMask != orig) {
        DBG_SER(1, "ClearLimitSwitchEvent: MER_MASK 0x%04X → 0x%04X", orig, merMask);
        g_activeCh->writeData16(merMaskAddr, merMask);
    }

    WORD merAddr = resolveAddr("MER", TML_DM_MER);
    WORD mer = 0;
    g_activeCh->readData16(merAddr, mer);
    WORD newMer = mer & ~(WORD)((1 << 6) | (1 << 7));
    if (newMer != mer) {
        DBG_SER(1, "ClearLimitSwitchEvent: MER 0x%04X → 0x%04X", mer, newMer);
        g_activeCh->writeData16(merAddr, newMer);
    }

    return TRUE;
}

BOOL TS_DisableLimitProtection(BOOL disableLSP, BOOL disableLSN)
{
    if (!g_activeCh) {
        setGlobalError("TS_DisableLimitProtection: no active channel");
        return FALSE;
    }

    WORD merMaskAddr = resolveAddr("MER_MASK", TML_DM_MER_MASK);
    WORD merMask = 0;
    if (!g_activeCh->readData16(merMaskAddr, merMask)) {
        merMask = 0;  /* safe default: all events disabled */
    }

    WORD orig = merMask;
    if (disableLSP)
        merMask &= ~(WORD)(1 << 6);  /* clear LSP event bit */
    if (disableLSN)
        merMask &= ~(WORD)(1 << 7);  /* clear LSN event bit */

    DBG_SER(1, "DisableLimitProtection: MER_MASK 0x%04X → 0x%04X (disLSP=%d disLSN=%d)",
            orig, merMask, (int)disableLSP, (int)disableLSN);

    if (!g_activeCh->writeData16(merMaskAddr, merMask)) {
        strncpy(g_lastError, g_activeCh->lastError(), sizeof(g_lastError) - 1);
        return FALSE;
    }

    /* Also clear any already-latched limit bits in MER so the drive
     * doesn't keep blocking motion based on stale state */
    WORD merAddr = resolveAddr("MER", TML_DM_MER);
    WORD mer = 0;
    g_activeCh->readData16(merAddr, mer);
    if (mer != 0) {
        WORD newMer = mer;
        if (disableLSP) newMer &= ~(WORD)(1 << 6);
        if (disableLSN) newMer &= ~(WORD)(1 << 7);
        if (newMer != mer) {
            DBG_SER(1, "DisableLimitProtection: clearing MER 0x%04X → 0x%04X", mer, newMer);
            g_activeCh->writeData16(merAddr, newMer);
        }
    }

    return TRUE;
}

} /* extern "C" */
