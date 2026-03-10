/*
 * drvTmlMotor.cpp
 *
 * EPICS asynMotorController / asynMotorAxis driver for Technosoft
 * intelligent drives.
 *
 * Architecture
 * ------------
 *   TmlController  — one per RS-232/485/CAN/XPORT channel.
 *                    Owns the single TML channel fd shared by all axes.
 *   TmlAxis        — one per physical drive on the channel.
 *                    Each axis maintains its own TML serial setup.
 *
 * Recovery strategy: when the shared channel breaks, selectAxis()
 * reconnects it and reinits only the requesting axis.  Other axes
 * are lazily marked needsReinit_ and re-setup themselves on their
 * next poll cycle.  This avoids cascade TS_DriveInitialisation
 * storms that reset all drives and stop motion.
 *
 * The TML communication layer is NOT thread-safe; every call is
 * serialized through tmlLock_ (an epicsMutex in the controller).
 *
 * Author:  Andrea Michelotti — INFN-LNF
 * Date:    2026-02
 */

#include <cstdio>
#include <cstring>
#include <cmath>
#include <unistd.h>

#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsExport.h>
#include <iocsh.h>
#include <asynOctetSyncIO.h>

#include "tmlSerial.h"
#include "drvTmlMotor.h"

/* Global debug level — settable from iocsh via "var drvTmlDebug N" */
int drvTmlDebug = 0;
extern "C" { epicsExportAddress(int, drvTmlDebug); }

#define DBG(level, fmt, ...) \
    do { if (drvTmlDebug >= (level)) \
        printf("drvTml [%s:%d] " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); \
    } while(0)

/* Detect channel type from device path:
 *   Contains '/'  -> local serial device (PTY or /dev/ttyXXX) -> CHANNEL_RS232
 *   No '/'        -> IP or hostname (optionally "IP:port")    -> CHANNEL_XPORT_IP
 */
static int tmlChannelType(const char *path)
{
    return (strchr(path, '/') == nullptr) ? CHANNEL_XPORT_IP : CHANNEL_RS232;
}

/* Pass the full "host:port" string so connectTcp() can parse it. */
static void tmlDevName(const char *path, char *buf, size_t buflen)
{
    strncpy(buf, path, buflen - 1);
    buf[buflen - 1] = '\0';
}

/* SRL register bits (Technosoft TML Status Register Low) */
#define SRL_BIT_GFLT             (1 << 8)   /* Global fault          */
#define SRL_BIT_AFLT             (1 << 9)   /* Axis fault            */
#define SRL_BIT_MOTION_COMPLETE  (1 << 10)  /* Motion complete       */
#define SRL_BIT_AXIS_ON          (1 << 15)  /* Power stage ON        */

/* SRH register bits (Technosoft TML Status Register High) */
#define SRH_BIT_FERR             (1 << 0)   /* Following / position error */
#define SRH_BIT_OVERSPEED        (1 << 1)   /* Over-speed            */
#define SRH_BIT_OVERCURRENT      (1 << 2)   /* Over-current          */
#define SRH_BIT_I2T              (1 << 3)   /* I2T protection limit  */
#define SRH_BIT_OVERTEMP         (1 << 4)   /* Over-temperature      */
#define SRH_BIT_UVOLT            (1 << 5)   /* Under-voltage         */
#define SRH_BIT_OVOLT            (1 << 6)   /* Over-voltage          */
#define SRH_BIT_FAULT            (1 << 15)  /* General fault (OR)    */

/* MER register bits (Technosoft TML Motion Error Register) */
#define MER_BIT_FERR             (1 << 0)   /* Following error       */
#define MER_BIT_LSP              (1 << 6)   /* Positive limit switch */
#define MER_BIT_LSN              (1 << 7)   /* Negative limit switch */
#define MER_BIT_HOME             (1 << 8)   /* Home switch active    */

/* ================================================================= */
/*                       TmlController                                */
/* ================================================================= */

static const char *driverName = "TmlController";

TmlController::TmlController(const char *portName, const char *devicePath,
                             int numAxes, int hostId, int baudRate,
                             double movingPoll, double idlePoll)
    : asynMotorController(portName, numAxes,
                          NUM_TML_PARAMS,  /* extra asyn params */
                          0,  /* additional interfaces (none) */
                          0,  /* additional interrupt interfaces */
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,  /* autoConnect */
                          0, 0)  /* priority, stackSize (defaults) */
    , channelFd_(-1)
    , hostId_(hostId)
    , baudRate_(baudRate > 0 ? baudRate : 9600)
{
    strncpy(devicePath_, devicePath, sizeof(devicePath_) - 1);
    devicePath_[sizeof(devicePath_) - 1] = '\0';
    memset(&lastReconnect_, 0, sizeof(lastReconnect_));

    /* Create extra parameters */
    createParam(TML_SRH_String,          asynParamInt32,    &tmlSRH_);
    createParam(TML_SRL_String,          asynParamInt32,    &tmlSRL_);
    createParam(TML_MER_String,          asynParamInt32,    &tmlMER_);
    createParam(TML_MCR_String,          asynParamInt32,    &tmlMCR_);
    createParam(TML_MSR_String,          asynParamInt32,    &tmlMSR_);
    createParam(TML_ISR_String,          asynParamInt32,    &tmlISR_);
    createParam(TML_SETUP_FILE_String,   asynParamOctet,    &tmlSetupFile_);
    createParam(TML_ACTIVE_String,       asynParamInt32,    &tmlActive_);
    createParam(TML_FAULT_TEXT_String,   asynParamOctet,    &tmlFaultText_);
    createParam(TML_APOS_String,         asynParamFloat64,  &tmlAPOS_);
    createParam(TML_CSPD_String,         asynParamFloat64,  &tmlCSPD_);
    createParam(TML_RESET_FAULT_String,  asynParamInt32,    &tmlResetFault_);
    createParam(TML_SAVE_EEPROM_String,  asynParamInt32,    &tmlSaveEeprom_);
    createParam(TML_RESET_DRIVE_String,  asynParamInt32,    &tmlResetDrive_);
    createParam(TML_POTM_String,         asynParamFloat64,  &tmlPOTM_);

    /* Open the TML communication channel (shared by all axes) */
    char devName[256];
    tmlDevName(devicePath_, devName, sizeof(devName));
    int chType = tmlChannelType(devicePath_);
    DBG(1, "Opening channel '%s' (devName='%s' type=%s) hostId=%d baud=%d",
        devicePath_, devName,
        chType == CHANNEL_XPORT_IP ? "XPORT_IP" : "RS232",
        hostId_, baudRate_);

    tmlLock_.lock();
    channelFd_ = TS_OpenChannel(devName, (BYTE)chType, (BYTE)hostId_, (DWORD)baudRate_);
    tmlLock_.unlock();

    if (channelFd_ < 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: TS_OpenChannel('%s', type=%d) FAILED: %s\n",
                  driverName, devName, chType, TS_GetLastErrorText());
    } else {
        DBG(1, "Channel opened, fd=%d type=%s", channelFd_,
            chType == CHANNEL_XPORT_IP ? "XPORT_IP" : "RS232");
    }

    /* Create axis objects (un-configured; user calls TmlAxisConfig for each) */
    for (int i = 0; i < numAxes; i++) {
        new TmlAxis(this, i);
    }

    /* Start the poller */
    startPoller(movingPoll, idlePoll, 2);
}

TmlController::~TmlController()
{
    tmlLock_.lock();
    if (channelFd_ >= 0)
        TS_CloseChannel(channelFd_);
    tmlLock_.unlock();
}

TmlAxis *TmlController::getTmlAxis(int axisNo)
{
    return static_cast<TmlAxis *>(asynMotorController::getAxis(axisNo));
}

TmlAxis *TmlController::getTmlAxis(asynUser *pasynUser)
{
    return static_cast<TmlAxis *>(asynMotorController::getAxis(pasynUser));
}

asynStatus TmlController::configAxis(int axisNo, int axisId,
                                     const char *setupFile,
                                     const char *homingSwitch,
                                     int ignoreLSP, int ignoreLSN,
                                     int scrValue)
{
    TmlAxis *pAxis = getTmlAxis(axisNo);
    if (!pAxis) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: axis %d does not exist\n", driverName, axisNo);
        return asynError;
    }
    return pAxis->configure(axisId, setupFile, homingSwitch,
                            ignoreLSP != 0, ignoreLSN != 0, scrValue);
}

void TmlController::report(FILE *fp, int level)
{
    fprintf(fp, "Technosoft TML motor controller '%s'\n", portName);
    fprintf(fp, "  Device path : %s\n", devicePath_);
    fprintf(fp, "  Channel fd  : %d\n", channelFd_);
    fprintf(fp, "  Host ID     : %d\n", hostId_);
    fprintf(fp, "  Baud rate   : %d\n", baudRate_);
    fprintf(fp, "  Num axes    : %d\n", numAxes_);

    asynMotorController::report(fp, level);
}

/* ---- writeInt32: handle command parameters ---- */
asynStatus TmlController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int axisNo   = 0;
    getAddress(pasynUser, &axisNo);
    TmlAxis *pAxis = getTmlAxis(axisNo);

    if (function == tmlResetFault_ && value && pAxis) {
        DBG(1, "Axis %d: Reset Fault command", axisNo);
        tmlLock_.lock();
        if (pAxis->selectAxis() == asynSuccess)
            TS_ResetFault();
        tmlLock_.unlock();
        return asynSuccess;
    }
    if (function == tmlSaveEeprom_ && value && pAxis) {
        DBG(1, "Axis %d: Save to EEPROM command", axisNo);
        tmlLock_.lock();
        if (pAxis->selectAxis() == asynSuccess)
            TS_Save();
        tmlLock_.unlock();
        return asynSuccess;
    }
    if (function == tmlResetDrive_ && value && pAxis) {
        DBG(1, "Axis %d: Reset Drive command", axisNo);
        tmlLock_.lock();
        if (pAxis->selectAxis() == asynSuccess)
            TS_Reset();
        tmlLock_.unlock();
        return asynSuccess;
    }

    return asynMotorController::writeInt32(pasynUser, value);
}

/* ================================================================= */
/*                            TmlAxis                                 */
/* ================================================================= */

TmlAxis::TmlAxis(TmlController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo)
    , pC_(pC)
    , axisId_(0)
    , setupIdx_(-1)
    , configured_(false)
    , activated_(false)
    , powered_(false)
    , homingActive_(false)
    , homingMoveSeen_(false)
    , stopping_(false)
    , useLSP_(false)
    , ignoreLSP_(false)
    , ignoreLSN_(false)
    , scrValue_(0)
    , needsReinit_(false)
    , reinitCountdown_(0)
    , reinitBackoff_(1)
    , pollCount_(0)
{
    memset(setupFile_, 0, sizeof(setupFile_));
    /* Publish initial inactive state */
    setIntegerParam(pC_->tmlActive_, 0);
    setStringParam (pC_->tmlFaultText_, "Not initialized");
    callParamCallbacks();
}

TmlAxis::~TmlAxis()
{
    if (configured_ && pC_->channelFd_ >= 0) {
        pC_->tmlLock_.lock();
        TS_SelectChannel(pC_->channelFd_);
        TS_SelectAxis((BYTE)axisId_);
        TS_Power(FALSE);
        pC_->tmlLock_.unlock();
    }
}

/* ---- configure ---- */
asynStatus TmlAxis::configure(int axisId, const char *setupFile,
                               const char *homingSwitch,
                               bool ignoreLSP, bool ignoreLSN,
                               int scrValue)
{
    if (pC_->channelFd_ < 0) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis::configure: channel not open\n");
        return asynError;
    }

    axisId_ = axisId;
    ignoreLSP_ = ignoreLSP;
    ignoreLSN_ = ignoreLSN;
    scrValue_ = scrValue;
    strncpy(setupFile_, setupFile, sizeof(setupFile_) - 1);

    if (homingSwitch && (strcmp(homingSwitch, "LSP") == 0 ||
                         strcmp(homingSwitch, "lsp") == 0))
        useLSP_ = true;
    else
        useLSP_ = false;

    if (ignoreLSP_ || ignoreLSN_)
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,
                  "TmlAxis[%d]: limit switch reporting: ignoreLSP=%d ignoreLSN=%d\n",
                  axisNo_, (int)ignoreLSP_, (int)ignoreLSN_);

    pC_->tmlLock_.lock();

    /* Mark as configured INSIDE the lock so the reconnect logic in
     * selectAxis() always sees this axis.  Without this, a reconnect
     * triggered by the poller between unlock() and the assignment
     * would skip this axis and its TML_lib setup would be lost. */
    configured_ = true;

    /* Select our channel */
    TS_SelectChannel(pC_->channelFd_);

    asynStatus st = reinitAxis();
    if (st != asynSuccess) {
        activated_  = false;
        needsReinit_ = true;
    } else {
        activated_  = true;
        needsReinit_ = false;
    }
    pC_->tmlLock_.unlock();

    if (st != asynSuccess) {
        /* Even if init failed (drive unresponsive), the axis is already
         * marked configured so the poll loop can re-attempt initialisation.
         * This prevents the IOC from refusing to start when the drive
         * is temporarily offline or the XPORT needs more time. */
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: init failed, will retry in poll loop: %s\n",
                  axisNo_, TS_GetLastErrorText());
        pC_->setStringParam(axisNo_, pC_->tmlSetupFile_, setupFile_);
        setIntegerParam(pC_->tmlActive_, 0);
        setStringParam(pC_->tmlFaultText_, "Drive not responding — will retry");
        callParamCallbacks();
        return asynSuccess;   /* don't block IOC startup */
    }
    /* Note: power stage is enabled lazily on first move via powerOn().
     * Calling TS_Power(TRUE) here can cause XPORT TCP connection resets
     * which invalidate the channel fd before the poller thread starts. */

    /* Publish setup file and active state */
    pC_->setStringParam(axisNo_, pC_->tmlSetupFile_, setupFile_);
    setIntegerParam(pC_->tmlActive_, 1);
    callParamCallbacks();

    DBG(1, "Axis %d configured+activated: TML-ID=%d setup='%s' homing=%s",
        axisNo_, axisId_, setupFile_, useLSP_ ? "LSP" : "LSN");

    return asynSuccess;
}

/* ---- reinitAxis: replay full TML setup on the current channel ----
 * Must be called with tmlLock_ held and channel already selected.     */
asynStatus TmlAxis::reinitAxis()
{
    /* Reuse existing setup slot if already loaded — avoids slot leak
     * on repeated retries.  Only load fresh on first call. */
    if (setupIdx_ < 0) {
        setupIdx_ = TS_LoadSetup(setupFile_);
        if (setupIdx_ < 0) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                      "TmlAxis[%d]: reinit TS_LoadSetup('%s') FAILED: %s\n",
                      axisNo_, setupFile_, TS_GetLastErrorText());
            return asynError;
        }
    }

    if (!TS_SetupAxis((BYTE)axisId_, setupIdx_)) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: reinit TS_SetupAxis(%d,%d) FAILED: %s\n",
                  axisNo_, axisId_, setupIdx_, TS_GetLastErrorText());
        return asynError;
    }

    if (!TS_SelectAxis((BYTE)axisId_)) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: reinit TS_SelectAxis(%d) FAILED: %s\n",
                  axisNo_, axisId_, TS_GetLastErrorText());
        return asynError;
    }

    if (!TS_DriveInitialisation()) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: reinit TS_DriveInitialisation FAILED: %s\n",
                  axisNo_, TS_GetLastErrorText());
        return asynError;
    }

    /* Post-ENDINIT diagnostic: read MER and MER_MASK to see the drive's
     * limit switch state and event configuration.
     * MER bit6=LSP (positive LS), bit7=LSN (negative LS).
     * Active inputs mean the firmware will block motion in that direction. */
    {
        WORD mer_w = 0;
        short merMaskVal = 0;
        TS_ReadStatus(REG_MER, mer_w);
        TS_GetIntVariable("MER_MASK", merMaskVal);

        bool lspActive = (mer_w & (1 << 6)) != 0;
        bool lsnActive = (mer_w & (1 << 7)) != 0;
        DBG(1, "Axis %d: post-ENDINIT MER=0x%04X MER_MASK=0x%04X %s%s",
            axisNo_, (int)mer_w, (int)(unsigned short)merMaskVal,
            (lspActive || lsnActive) ? "LS active: " : "no LS active",
            (lspActive || lsnActive) ?
                (lspActive && lsnActive ? "LSP(+) LSN(-)" :
                 lspActive ? "LSP(+)" : "LSN(-)") : "");
        if (lspActive || lsnActive) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_WARNING,
                      "TmlAxis[%d]: WARNING post-ENDINIT: MER=0x%04X — "
                      "limit switch inputs active at boot: %s%s\n"
                      "  If motor is NOT at a physical limit, check LS wiring "
                      "polarity (NC/NO switch type).\n",
                      axisNo_, (int)mer_w,
                      lspActive ? "LSP(+) " : "",
                      lsnActive ? "LSN(-) " : "");
        }
    }

    /* Configure encoder input via SCR (Setup Configuration Register).
     * Without this, APOS stays zero because the drive does not read the
     * encoder.  The SCR value is drive-specific; 0x4338 enables the
     * primary encoder with open-loop position feedback.
     * The SCR address (typically 0x0300) is resolved from the .t.zip
     * setup file's variable map. */
    if (scrValue_ != 0) {
        if (!TS_SetIntVariable("SCR", (short)scrValue_)) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                      "TmlAxis[%d]: SCR write (0x%04X) FAILED: %s\n",
                      axisNo_, scrValue_, TS_GetLastErrorText());
            /* Not fatal — encoder readback won't work but motion may still run */
        } else {
            DBG(1, "Axis %d: SCR=0x%04X written", axisNo_, scrValue_);
        }

        /* Verify APOS is readable after SCR configuration */
        long aposCheck = 0;
        if (TS_GetLongVariable("APOS", aposCheck)) {
            DBG(1, "Axis %d: post-SCR APOS=%ld (encoder feedback configured)",
                axisNo_, aposCheck);
        } else {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                      "TmlAxis[%d]: post-SCR APOS read FAILED: %s\n",
                      axisNo_, TS_GetLastErrorText());
        }
    }

    /* Disable drive-level limit switch protection for unconnected inputs.
     * Without this, the drive firmware blocks motion in the corresponding
     * direction even though the limit switch input is just floating. */
    if (ignoreLSP_ || ignoreLSN_) {
        if (!TS_DisableLimitProtection(ignoreLSP_ ? TRUE : FALSE,
                                       ignoreLSN_ ? TRUE : FALSE)) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                      "TmlAxis[%d]: TS_DisableLimitProtection FAILED: %s\n",
                      axisNo_, TS_GetLastErrorText());
            /* Not fatal — continue anyway */
        }
    }

    DBG(1, "Axis %d reinit OK: TML-ID=%d setupIdx=%d", axisNo_, axisId_, setupIdx_);
    return asynSuccess;
}

/* ---- selectAxis helper ----
 *
 * Shared channel with lazy per-axis recovery:
 *   All axes share one TML channel (TML_lib only allows one
 *   TS_OpenChannel per physical serial port).  When the channel
 *   breaks, selectAxis() reconnects it ONCE and reinits only
 *   the requesting axis.  Other axes are marked needsReinit_
 *   and re-setup themselves lazily on their next poll cycle.
 *   This avoids the cascade TS_DriveInitialisation storm that
 *   resets all drives and stops motion.
 *
 *   Tier 1  Retry TS_SelectChannel up to 3 times with short delays.
 *           Handles transient timeouts through socat/Moxa.
 *   Tier 2  Rate-limit: if we reconnected within the last 3 s,
 *           skip the expensive close/reopen/reinit cycle and
 *           just return asynError so the caller can retry later.
 *   Tier 3  Full reconnect: close channel, reopen, reinit only
 *           THIS axis.  Mark all other axes needsReinit_ for lazy
 *           recovery on their next poll.
 */
asynStatus TmlAxis::selectAxis()
{
    if (!configured_ || pC_->channelFd_ < 0)
        return asynError;

    /* ---- Tier 1: retry TS_SelectChannel (transient failures) ---- */
    bool channelOk = false;
    for (int retry = 0; retry < 3; retry++) {
        if (TS_SelectChannel(pC_->channelFd_)) {
            channelOk = true;
            break;
        }
        if (retry < 2) {
            DBG(1, "TS_SelectChannel(%d) axis %d retry %d/3",
                pC_->channelFd_, axisNo_, retry + 1);
            epicsThreadSleep(0.05);   /* 50 ms between retries */
        }
    }

    if (!channelOk) {
        /* ---- Tier 2: reconnect cooldown ---- */
        epicsTimeStamp now;
        epicsTimeGetCurrent(&now);
        double elapsed = epicsTimeDiffInSeconds(&now, &pC_->lastReconnect_);
        if (elapsed < 3.0 && pC_->lastReconnect_.secPastEpoch > 0) {
            DBG(1, "TS_SelectChannel(%d) axis %d failed (%.1fs since last reconnect — cooldown)",
                pC_->channelFd_, axisNo_, elapsed);
            return asynError;
        }

        /* ---- Tier 3: full reconnect ---- */
        DBG(0, "Axis %d: TS_SelectChannel(%d) failed (%s) — reconnecting",
            axisNo_, pC_->channelFd_, TS_GetLastErrorText());
        TS_CloseChannel(pC_->channelFd_);
        char devName[256];
        tmlDevName(pC_->devicePath_, devName, sizeof(devName));
        int chType = tmlChannelType(pC_->devicePath_);
        int newFd = TS_OpenChannel(devName, (BYTE)chType,
                                   (BYTE)pC_->hostId_, (DWORD)pC_->baudRate_);
        if (newFd < 0) {
            DBG(0, "Axis %d: reconnect failed: %s", axisNo_, TS_GetLastErrorText());
            pC_->channelFd_ = -1;
            epicsTimeGetCurrent(&pC_->lastReconnect_);
            return asynError;
        }
        pC_->channelFd_ = newFd;
        epicsTimeGetCurrent(&pC_->lastReconnect_);
        DBG(0, "Axis %d: reconnected, new fd=%d", axisNo_, newFd);
        if (!TS_SelectChannel(pC_->channelFd_)) {
            DBG(0, "Axis %d: TS_SelectChannel after reconnect failed: %s",
                axisNo_, TS_GetLastErrorText());
            return asynError;
        }

        /* Mark ALL OTHER axes as needing lazy reinit on their next poll.
         * Do NOT call reinitAxis() / TS_DriveInitialisation on them now —
         * that would reset drives and stop motion (the cascade storm). */
        for (int ax = 0; ax < pC_->numAxes_; ax++) {
            TmlAxis *pAx = pC_->getTmlAxis(ax);
            if (!pAx || !pAx->configured_ || ax == axisNo_) continue;
            pAx->setupIdx_        = -1;   /* force fresh TS_LoadSetup */
            pAx->needsReinit_     = true;
            pAx->reinitCountdown_ = 0;    /* reinit on next poll */
            pAx->reinitBackoff_   = 1;
            pAx->activated_       = false;
            pAx->powered_         = false;
            pAx->stopping_        = false;
            pAx->setIntegerParam(pC_->tmlActive_, 0);
            pAx->callParamCallbacks();
            DBG(1, "Axis %d: marked for lazy reinit after channel reconnect", ax);
        }

        /* Reinit THIS axis immediately so we can proceed */
        setupIdx_ = -1;   /* force fresh TS_LoadSetup on new channel */
        if (reinitAxis() != asynSuccess) {
            DBG(0, "Axis %d: reinitAxis after reconnect FAILED", axisNo_);
            activated_       = false;
            powered_         = false;
            needsReinit_     = true;
            reinitCountdown_ = 1;
            reinitBackoff_   = 1;
            setIntegerParam(pC_->tmlActive_, 0);
            callParamCallbacks();
            return asynError;
        }

        DBG(0, "Axis %d: reinitAxis after reconnect OK (fd=%d)", axisNo_, newFd);
        powered_      = false;  /* power stage is off after reinit */
        stopping_     = false;
        needsReinit_  = false;
        activated_    = true;
        setIntegerParam(pC_->tmlActive_, 1);
        callParamCallbacks();
        return asynSuccess;
    }

    if (!TS_SelectAxis((BYTE)axisId_)) {
        DBG(0, "Axis %d: TS_SelectAxis(%d) failed: %s",
            axisNo_, axisId_, TS_GetLastErrorText());
        /* TML_lib setup for this axis was lost (e.g. channel reconnect
         * by another axis, or internal library state corruption).
         * Flag for lazy reinit so the poll loop retries. */
        needsReinit_  = true;
        activated_    = false;
        reinitCountdown_ = 0;
        reinitBackoff_   = 1;
        return asynError;
    }
    return asynSuccess;
}

/* ---- power helpers ---- */
asynStatus TmlAxis::powerOn()
{
    if (powered_) return asynSuccess;
    if (selectAxis() != asynSuccess) return asynError;

    if (!TS_Power(TRUE)) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: TS_Power(ON) FAILED: %s\n",
                  axisNo_, TS_GetLastErrorText());
        return asynError;
    }

    /* Wait for the power stage to actually turn on (SRL bit 15).
     * Keep tmlLock_ held throughout so the poller cannot trigger a
     * reconnect (and TS_DriveInitialisation on all axes) while
     * we are waiting for the power stage to stabilise. */
    for (int i = 0; i < 50; i++) {   /* up to 5 seconds */
        WORD srl = 0;
        TS_ReadStatus(REG_SRL, srl);
        if (srl & SRL_BIT_AXIS_ON) {
            powered_ = true;
            activated_ = true;
            setIntegerParam(pC_->tmlActive_, 1);
            callParamCallbacks();
            DBG(2, "Axis %d powered ON", axisNo_);
            return asynSuccess;
        }
        epicsThreadSleep(0.1);
    }

    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "TmlAxis[%d]: power-on timeout\n", axisNo_);
    return asynError;
}

asynStatus TmlAxis::powerOff()
{
    if (!powered_) return asynSuccess;
    if (selectAxis() != asynSuccess) return asynError;

    TS_Power(FALSE);
    powered_ = false;
    setIntegerParam(pC_->tmlActive_, 0);
    callParamCallbacks();
    DBG(2, "Axis %d powered OFF", axisNo_);
    return asynSuccess;
}

/* ================================================================= */
/*                        Motion commands                             */
/* ================================================================= */

asynStatus TmlAxis::move(double position, int relative, double minVelocity,
                         double maxVelocity, double acceleration)
{
    if (!configured_) return asynError;

    DBG(1, "CMD Axis %d: move %s pos=%.0f vel=%.2f acc=%.2f (.VAL write)",
        axisNo_, relative ? "REL" : "ABS", position, maxVelocity, acceleration);

    pC_->tmlLock_.lock();
    asynStatus st = selectAxis();
    if (st != asynSuccess) {
        DBG(1, "CMD Axis %d: move ABORTED — selectAxis failed", axisNo_);
        pC_->tmlLock_.unlock(); return st;
    }

    st = powerOn();
    if (st != asynSuccess) {
        DBG(1, "CMD Axis %d: move ABORTED — powerOn failed", axisNo_);
        pC_->tmlLock_.unlock(); return st;
    }

    homingActive_ = false;
    stopping_     = false;

    /* Disarm any limit-switch event left armed by a previous home() call.
     * If MER_MASK still has bit6/7 set and the LS input is HIGH, the drive
     * applies an immediate stop right after UPD, giving MC=1 before the
     * motor moves.  ClearLimitSwitchEvent is a no-op when MER_MASK is already 0. */
    TS_ClearLimitSwitchEvent();

    BOOL ok;
    long pos = (long)position;

    if (relative) {
        ok = TS_MoveRelative(pos, maxVelocity, acceleration,
                             FALSE, UPDATE_IMMEDIATE, FROM_REFERENCE);
    } else {
        ok = TS_MoveAbsolute(pos, maxVelocity, acceleration,
                             UPDATE_IMMEDIATE, FROM_REFERENCE);
    }

    if (ok) {
        TS_SetEventOnMotionComplete(TRUE, TRUE);
    }

    pC_->tmlLock_.unlock();

    if (!ok) {
        DBG(1, "CMD Axis %d: move FAILED — %s", axisNo_, TS_GetLastErrorText());
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: move FAILED: %s\n",
                  axisNo_, TS_GetLastErrorText());
        return asynError;
    }

    DBG(1, "CMD Axis %d: move %s pos=%ld OK — motion started",
        axisNo_, relative ? "REL" : "ABS", pos);
    setIntegerParam(pC_->motorStatusDone_, 0);
    setIntegerParam(pC_->motorStatusMoving_, 1);
    callParamCallbacks();

    return asynSuccess;
}

asynStatus TmlAxis::moveVelocity(double minVelocity, double maxVelocity,
                                 double acceleration)
{
    if (!configured_) return asynError;

    DBG(1, "CMD Axis %d: jog vel=%.2f acc=%.2f (.JOGF/.JOGR write)",
        axisNo_, maxVelocity, acceleration);

    pC_->tmlLock_.lock();
    asynStatus st = selectAxis();
    if (st != asynSuccess) {
        DBG(1, "CMD Axis %d: jog ABORTED — selectAxis failed", axisNo_);
        pC_->tmlLock_.unlock(); return st;
    }

    st = powerOn();
    if (st != asynSuccess) {
        DBG(1, "CMD Axis %d: jog ABORTED — powerOn failed", axisNo_);
        pC_->tmlLock_.unlock(); return st;
    }

    homingActive_ = false;
    stopping_     = false;

    /* Disarm any stale LS event from a previous home() before starting jog */
    TS_ClearLimitSwitchEvent();

    BOOL ok = TS_MoveVelocity(maxVelocity, acceleration,
                              UPDATE_IMMEDIATE, FROM_REFERENCE);
    pC_->tmlLock_.unlock();

    if (!ok) {
        DBG(1, "CMD Axis %d: jog FAILED — %s", axisNo_, TS_GetLastErrorText());
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: moveVelocity FAILED: %s\n",
                  axisNo_, TS_GetLastErrorText());
        return asynError;
    }

    DBG(1, "CMD Axis %d: jog %s vel=%.2f OK — motion started",
        axisNo_, maxVelocity > 0 ? "FWD" : "REV", maxVelocity);
    setIntegerParam(pC_->motorStatusDone_, 0);
    setIntegerParam(pC_->motorStatusMoving_, 1);
    callParamCallbacks();

    return asynSuccess;
}

asynStatus TmlAxis::home(double minVelocity, double maxVelocity,
                         double acceleration, int forwards)
{
    if (!configured_) return asynError;

    /* Guard: if HVEL is 0, fall back to VELO from the motor record */
    if (maxVelocity == 0.0) {
        double velo = 0;
        pC_->getDoubleParam(axisNo_, pC_->motorVelocity_, &velo);
        if (velo > 0) {
            maxVelocity = velo;
            DBG(1, "CMD Axis %d: home — HVEL=0, using VELO=%.2f instead",
                axisNo_, maxVelocity);
        } else {
            DBG(1, "CMD Axis %d: home REJECTED — both HVEL and VELO are 0",
                axisNo_);
            return asynError;
        }
    }
    if (acceleration == 0.0) {
        acceleration = 1.0;  /* safe default: 1 EGU/s² */
        DBG(1, "CMD Axis %d: home — acc=0, using default 1.0", axisNo_);
    }

    DBG(1, "CMD Axis %d: home %s vel=%.2f acc=%.2f (.HOM%c write)",
        axisNo_, forwards ? "FORWARD(LSP)" : "REVERSE(LSN)",
        maxVelocity, acceleration, forwards ? 'F' : 'R');

    pC_->tmlLock_.lock();
    asynStatus st = selectAxis();
    if (st != asynSuccess) {
        DBG(1, "CMD Axis %d: home ABORTED — selectAxis failed", axisNo_);
        pC_->tmlLock_.unlock(); return st;
    }

    st = powerOn();
    if (st != asynSuccess) {
        DBG(1, "CMD Axis %d: home ABORTED — powerOn failed", axisNo_);
        pC_->tmlLock_.unlock(); return st;
    }

    /*
     * Homing strategy:
     *   - HOMF (forwards=1): move towards the POSITIVE limit switch (LSP)
     *   - HOMR (forwards=0): move towards the NEGATIVE limit switch (LSN)
     *   1. Start a velocity move towards the chosen limit switch.
     *   2. Set an event on that limit switch.
     *   3. The poller detects motion-complete when the LS fires.
     *   4. Position is set to 0 at the home position.
     */
    short lsType;
    double speed;

    if (forwards) {
        speed  = fabs(maxVelocity);
        lsType = LSW_POSITIVE;
    } else {
        speed  = -fabs(maxVelocity);
        lsType = LSW_NEGATIVE;
    }

    /* Clear any stale LSP/LSN latch in MER (and the MER_MASK event bits)
     * before arming the new event.  If MER already has the target limit-switch
     * bit set when we call TS_SetEventOnLimitSwitch, the drive's event
     * condition is immediately satisfied and MC fires before the motor moves. */
    TS_ClearLimitSwitchEvent();

    BOOL ok = TS_MoveVelocity(speed, fabs(acceleration),
                              UPDATE_IMMEDIATE, FROM_REFERENCE);
    if (ok) {
        ok = TS_SetEventOnLimitSwitch(lsType, TRANSITION_LOW_TO_HIGH, TRUE, TRUE);
    }

    pC_->tmlLock_.unlock();

    if (!ok) {
        DBG(1, "CMD Axis %d: home FAILED — %s", axisNo_, TS_GetLastErrorText());
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: home FAILED: %s\n",
                  axisNo_, TS_GetLastErrorText());
        return asynError;
    }

    DBG(1, "CMD Axis %d: home %s OK — homing started",
        axisNo_, forwards ? "FORWARD(LSP)" : "REVERSE(LSN)");
    homingActive_    = true;
    homingMoveSeen_  = false;  /* guard: require actual movement before completion */
    stopping_        = false;
    setIntegerParam(pC_->motorStatusDone_, 0);
    setIntegerParam(pC_->motorStatusMoving_, 1);
    setIntegerParam(pC_->motorStatusHomed_, 0);
    callParamCallbacks();

    return asynSuccess;
}

asynStatus TmlAxis::stop(double acceleration)
{
    if (!configured_) return asynError;

    DBG(1, "CMD Axis %d: stop (.STOP write)", axisNo_);

    pC_->tmlLock_.lock();
    selectAxis();

    /* STOP0 (immediate): forces motor voltage to 0.  This is the only
     * stop mode guaranteed to work in ALL drive configurations including
     * open-loop (stepper) where the speed loop is not closed.
     *
     * Do NOT follow with TS_Stop() (STOP3): STOP3 overrides STOP0 by
     * switching to speed-profile deceleration, which requires a closed
     * speed loop.  In open-loop mode STOP3 only stops the reference
     * generator (TPOS) while the physical motor keeps running. */
    TS_ABORT();

    /* Disarm LS event if homing was in progress — without this, the next
     * move command would be immediately blocked by the still-armed MER_MASK */
    if (homingActive_)
        TS_ClearLimitSwitchEvent();

    pC_->tmlLock_.unlock();

    homingActive_ = false;
    stopping_     = true;  /* hold MOVN=0 until hardware confirms MC */
    setIntegerParam(pC_->motorStatusDone_, 1);
    setIntegerParam(pC_->motorStatusMoving_, 0);
    callParamCallbacks();

    return asynSuccess;
}

asynStatus TmlAxis::setPosition(double position)
{
    if (!configured_) return asynError;

    DBG(1, "CMD Axis %d: setPosition %.0f (.SET write)", axisNo_, position);

    pC_->tmlLock_.lock();
    asynStatus st = selectAxis();
    if (st != asynSuccess) { pC_->tmlLock_.unlock(); return st; }

    BOOL ok = TS_SetPosition((long)position);
    pC_->tmlLock_.unlock();

    if (!ok) {
        DBG(1, "CMD Axis %d: setPosition FAILED — %s", axisNo_, TS_GetLastErrorText());
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: setPosition FAILED: %s\n",
                  axisNo_, TS_GetLastErrorText());
        return asynError;
    }

    DBG(1, "CMD Axis %d: setPosition %.0f OK", axisNo_, position);
    return asynSuccess;
}

/* ---- setClosedLoop: map to TML power stage ON/OFF ---- */
asynStatus TmlAxis::setClosedLoop(bool closedLoop)
{
    if (!configured_) return asynError;

    DBG(1, "CMD Axis %d: closedLoop %s (.CNEN write)", axisNo_, closedLoop ? "ON" : "OFF");

    pC_->tmlLock_.lock();
    asynStatus st;
    if (closedLoop) {
        st = powerOn();
    } else {
        st = powerOff();
    }
    pC_->tmlLock_.unlock();

    return st;
}

/* ================================================================= */
/*                            Polling                                 */
/* ================================================================= */

asynStatus TmlAxis::readRegisters(unsigned short &srh, unsigned short &srl,
                                  unsigned short &mer)
{
    WORD val;
    srh = srl = mer = 0;

    if (!TS_ReadStatus(REG_SRH, val)) return asynError;
    srh = val;
    if (!TS_ReadStatus(REG_SRL, val)) return asynError;
    srl = val;
    if (!TS_ReadStatus(REG_MER, val)) return asynError;
    mer = val;

    return asynSuccess;
}

asynStatus TmlAxis::poll(bool *moving)
{
    *moving = false;
    if (!configured_) return asynSuccess;

    /* If initialisation failed at startup, retry with exponential backoff */
    if (needsReinit_) {
        if (reinitCountdown_ > 0) {
            reinitCountdown_--;
            setIntegerParam(pC_->motorStatusCommsError_, 1);
            callParamCallbacks();
            return asynSuccess;  /* waiting for backoff to expire */
        }
        pC_->tmlLock_.lock();
        TS_SelectChannel(pC_->channelFd_);
        asynStatus rst = reinitAxis();
        pC_->tmlLock_.unlock();
        if (rst != asynSuccess) {
            /* Exponential backoff: 1, 2, 4, 8, ... up to 30 poll cycles
             * At 1s idle poll, this means 1s, 2s, 4s, 8s, 16s, 30s */
            reinitCountdown_ = reinitBackoff_;
            if (reinitBackoff_ < 30) reinitBackoff_ *= 2;
            if (reinitBackoff_ > 30) reinitBackoff_ = 30;
            DBG(1, "Axis %d: reinit failed, next retry in %d poll cycles",
                axisNo_, reinitCountdown_);
            setIntegerParam(pC_->motorStatusCommsError_, 1);
            callParamCallbacks();
            return asynSuccess;
        }
        needsReinit_ = false;
        activated_ = true;
        DBG(1, "Axis %d: deferred init succeeded", axisNo_);
        setIntegerParam(pC_->tmlActive_, 1);
        setIntegerParam(pC_->motorStatusCommsError_, 0);
        setStringParam(pC_->tmlFaultText_, "");
        callParamCallbacks();
    }

    pC_->tmlLock_.lock();
    asynStatus st = selectAxis();
    if (st != asynSuccess) {
        pC_->tmlLock_.unlock();
        /* Lost communication — mark axis as inactive */
        activated_ = false;
        powered_   = false;
        setIntegerParam(pC_->tmlActive_,             0);
        setIntegerParam(pC_->motorStatusCommsError_, 1);
        setIntegerParam(pC_->motorStatusPowerOn_,    0);
        callParamCallbacks();
        return asynError;
    }

    /* ---- Read position: TPOS (trajectory) and APOS (actual/encoder) ---- */
    long pos = 0, apos = 0;
    BOOL posOk  = TS_GetLongVariable("TPOS", pos);
    BOOL aposOk = TS_GetLongVariable("APOS", apos);

    /* Log APOS diagnostic on first successful poll */
    if (pollCount_ == 0) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,
                  "TmlAxis[%d]: first poll APOS read %s, value=%ld  TPOS=%ld\n",
                  axisNo_, aposOk ? "OK" : "FAILED", apos, pos);
        if (!aposOk) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                      "TmlAxis[%d]: WARNING — APOS read failed (%s). "
                      ".REP will mirror TPOS. Consider setting UEIP=No.\n",
                      axisNo_, TS_GetLastErrorText());
        }
    }

    /* ---- Read status registers: SRL, MER (minimum set for motion control) ---- */
    unsigned short srh = 0, srl = 0, mer = 0;
    WORD srl_w = 0, mer_w = 0;
    BOOL srlOk = TS_ReadStatus(REG_SRL, srl_w);
    BOOL merOk = TS_ReadStatus(REG_MER, mer_w);
    srl = (unsigned short)srl_w;
    mer = (unsigned short)mer_w;

    /* Only read SRH + extra registers when SRL shows a fault */
    bool srlFault = (srl & (SRL_BIT_GFLT | SRL_BIT_AFLT)) != 0;
    WORD mcr_w = 0, msr_w = 0, isr_w = 0, srh_w = 0;
    long cspd_raw = 0;
    short potm_raw = 0;
    if (srlFault) {
        TS_ReadStatus(REG_SRH, srh_w);
        srh = (unsigned short)srh_w;
    }
    /* Read MCR/MSR/ISR/CSPD/POTM only every 10th poll to reduce bus traffic */
    bool slowPoll = (pollCount_ % 10) == 0;
    if (slowPoll) {
        TS_ReadStatus(REG_MCR, mcr_w);
        TS_ReadStatus(REG_MSR, msr_w);
        TS_ReadStatus(REG_ISR, isr_w);
        TS_GetLongVariable("CSPD", cspd_raw);
        TS_GetIntVariable("POTM", potm_raw);
    }
    pollCount_++;

    pC_->tmlLock_.unlock();

    if (!posOk || !srlOk) {
        setIntegerParam(pC_->motorStatusCommsError_, 1);
        callParamCallbacks();
        return asynError;
    }

    /* Position: motorPosition_ = TPOS (trajectory generator output)
     *           motorEncoderPosition_ = APOS (actual encoder position)
     * The motor record uses UEIP to choose which one feeds RBV:
     *   UEIP=Yes → RBV = REP (encoder = APOS)
     *   UEIP=No  → RBV = RMP (motor   = TPOS)
     * Set UEIP=Yes in substitutions to use actual position. */
    setDoubleParam(pC_->motorPosition_, (double)pos);
    if (aposOk) {
        setDoubleParam(pC_->motorEncoderPosition_, (double)apos);
        setDoubleParam(pC_->tmlAPOS_, (double)apos);
    } else {
        /* APOS read failed — mirror TPOS into encoder position so that
         * .REP stays current even without a working encoder. */
        setDoubleParam(pC_->motorEncoderPosition_, (double)pos);
        setDoubleParam(pC_->tmlAPOS_, (double)pos);
    }

    /* Commanded speed (CSPD), POTM and MCR/MSR/ISR: only update on slow-poll
     * cycles to avoid overwriting with stale zeros on every fast poll. */
    if (slowPoll) {
        double cspd_val = (double)cspd_raw / 65536.0;
        setDoubleParam(pC_->tmlCSPD_, cspd_val);

        /* Potentiometer / ADC readback (POTM): 16-bit unsigned, 0-65535 */
        setDoubleParam(pC_->tmlPOTM_, (double)(unsigned short)potm_raw);

        setIntegerParam(pC_->tmlMCR_, (int)mcr_w);
        setIntegerParam(pC_->tmlMSR_, (int)msr_w);
        setIntegerParam(pC_->tmlISR_, (int)isr_w);
    }

    /* Publish raw registers */
    setIntegerParam(pC_->tmlSRH_, (int)srh);
    setIntegerParam(pC_->tmlSRL_, (int)srl);
    setIntegerParam(pC_->tmlMER_, (int)mer);

    /* ---- Map TML status to motor record status bits ---- */

    /* Motion complete = SRL bit 10 */
    bool motionComplete = (srl & SRL_BIT_MOTION_COMPLETE) != 0;

    /* If stop() was recently sent, hold motionComplete=true until the drive
     * firmware actually asserts MC, preventing MOVN from bouncing back. */
    if (stopping_) {
        if (motionComplete)
            stopping_ = false;   /* hardware confirmed; clear flag */
        else
            motionComplete = true;  /* override: report stopped */
    }
    bool axisON         = (srl & SRL_BIT_AXIS_ON) != 0;
    bool gflt           = (srl & SRL_BIT_GFLT) != 0;
    bool aflt           = (srl & SRL_BIT_AFLT) != 0;
    bool fault          = (srh & SRH_BIT_FAULT) != 0;
    bool ferr           = (srh & SRH_BIT_FERR) != 0;
    bool overspeed      = (srh & SRH_BIT_OVERSPEED) != 0;
    bool overcurrent    = (srh & SRH_BIT_OVERCURRENT) != 0;
    bool overtemp       = (srh & SRH_BIT_OVERTEMP) != 0;
    bool uvolt          = (srh & SRH_BIT_UVOLT) != 0;
    bool ovolt          = (srh & SRH_BIT_OVOLT) != 0;
    bool lsp            = (mer & MER_BIT_LSP) != 0;
    bool lsn            = (mer & MER_BIT_LSN) != 0;
    bool home           = (mer & MER_BIT_HOME) != 0;

    /* Build human-readable fault text */
    char faultBuf[128] = "OK";
    if (fault || gflt || aflt) {
        faultBuf[0] = '\0';
        if (ferr)        strncat(faultBuf, "FollowErr ",  sizeof(faultBuf)-strlen(faultBuf)-1);
        if (overspeed)   strncat(faultBuf, "Overspeed ",  sizeof(faultBuf)-strlen(faultBuf)-1);
        if (overcurrent) strncat(faultBuf, "Overcurrent ",sizeof(faultBuf)-strlen(faultBuf)-1);
        if (overtemp)    strncat(faultBuf, "Overtemp ",   sizeof(faultBuf)-strlen(faultBuf)-1);
        if (uvolt)       strncat(faultBuf, "UnderVolt ",  sizeof(faultBuf)-strlen(faultBuf)-1);
        if (ovolt)       strncat(faultBuf, "OverVolt ",   sizeof(faultBuf)-strlen(faultBuf)-1);
        if (gflt && !ferr && !overspeed && !overcurrent && !overtemp && !uvolt && !ovolt)
            strncat(faultBuf, "GlobalFault ", sizeof(faultBuf)-strlen(faultBuf)-1);
        if (aflt)        strncat(faultBuf, "AxisFault",   sizeof(faultBuf)-strlen(faultBuf)-1);
        /* trim trailing space */
        int len = strlen(faultBuf);
        if (len > 0 && faultBuf[len-1] == ' ') faultBuf[len-1] = '\0';
    }

    *moving = !motionComplete && axisON;

    /* Track that homing motion actually started (MC cleared at least once) */
    if (homingActive_ && *moving)
        homingMoveSeen_ = true;

    /* Sync powered_ / activated_ with hardware-reported AxisON state */
    if (axisON && !powered_) {
        powered_   = true;
        activated_ = true;
    } else if (!axisON && powered_) {
        powered_ = false;
    }

    setIntegerParam(pC_->motorStatusDone_,      motionComplete ? 1 : 0);
    setIntegerParam(pC_->motorStatusMoving_,     *moving ? 1 : 0);
    /* Limit switches: report the real MER state unless the corresponding
     * ignore flag is set (for floating / unconnected limit switch inputs).
     * NOTE: the motor record WILL raise MAJOR/STATE alarm when HLS or LLS
     * goes to 1 — this is hardcoded in the motor record and cannot be
     * suppressed.  Set ignoreLSP/ignoreLSN=1 in TmlAxisConfig for axes
     * where the limit switch input is not wired to avoid false alarms. */
    setIntegerParam(pC_->motorStatusHighLimit_,  (!ignoreLSP_ && lsp) ? 1 : 0);
    setIntegerParam(pC_->motorStatusLowLimit_,   (!ignoreLSN_ && lsn) ? 1 : 0);
    setIntegerParam(pC_->motorStatusPowerOn_,    axisON ? 1 : 0);
    setIntegerParam(pC_->motorStatusProblem_,    fault ? 1 : 0);
    setIntegerParam(pC_->motorStatusCommsError_, 0);
    /* Map missing MSTA bits */
    setIntegerParam(pC_->motorStatusFollowingError_, ferr ? 1 : 0);
    setIntegerParam(pC_->motorStatusHasEncoder_,     1);
    setIntegerParam(pC_->motorStatusGainSupport_,    1);
    /* Direction: positive = speed > 0, track from commanded speed sign */
    setIntegerParam(pC_->motorStatusDirection_,  (cspd_raw >= 0) ? 1 : 0);
    /* Reflect HOME switch state when not actively homing */
    if (!homingActive_)
        setIntegerParam(pC_->motorStatusAtHome_, home ? 1 : 0);
    /* TML_ACTIVE: axis is active when initialized AND power stage is on */
    setIntegerParam(pC_->tmlActive_,             (activated_ && axisON) ? 1 : 0);
    setStringParam (pC_->tmlFaultText_,          faultBuf);

    if (fault || gflt)
        DBG(1, "Axis %d FAULT: SRH=0x%04X SRL=0x%04X MER=0x%04X — %s",
            axisNo_, srh, srl, mer, faultBuf);

    /* Homing complete logic: motion-complete after we confirmed the drive
     * was actually moving (homingMoveSeen_).  This guards against two races:
     *   1. SRL.MC=1 when idle — first poll after home() starts still sees MC=1.
     *   2. Stale MER latch — pre-existing LSP/LSN bit would satisfy the event
     *      condition immediately if not cleared before TS_SetEventOnLimitSwitch.
     * Both are also addressed at the source (see home()), but the guard here
     * provides defence-in-depth. */
    if (homingActive_ && motionComplete && homingMoveSeen_) {
        /* Set absolute position to zero at home.
         * TML_lib: use TS_Execute("SAP 0") — the TML direct command that
         *   resets both target and actual position registers atomically.
         * Native:  use TS_SetPosition(0) — the native implementation's
         *   sendSAP() does the same thing under the hood.
         *
         * Also clear the limit switch event from MER_MASK that was set by
         * TS_SetEventOnLimitSwitch during home().  Without this, the MER
         * register latches the limit switch bit — making it appear sticky
         * even after the physical switch is released (MER_MASK enables
         * event detection which latches the bit; clearing it restores
         * live input state). */
        pC_->tmlLock_.lock();
        selectAxis();
        TS_SetPosition(0);
        TS_ClearLimitSwitchEvent();
        pC_->tmlLock_.unlock();

        homingActive_ = false;
        setIntegerParam(pC_->motorStatusHomed_, 1);
        setIntegerParam(pC_->motorStatusAtHome_, 1);
        setDoubleParam(pC_->motorPosition_, 0.0);
    }

    /* If motion complete and axis was powered, we can optionally power down.
       For now we leave the drive powered — typical for Technosoft setups. */

    callParamCallbacks();
    return asynSuccess;
}

void TmlAxis::report(FILE *fp, int level)
{
    fprintf(fp, "  Axis %d: TML-ID=%d configured=%s activated=%s setup='%s' powered=%s homing=%s home=%s\n",
            axisNo_, axisId_,
            configured_ ? "yes" : "no",
            activated_  ? "yes" : "no",
            setupFile_,
            powered_ ? "on" : "off",
            homingActive_ ? "active" : "idle",
            useLSP_ ? "LSP" : "LSN");

    asynMotorAxis::report(fp, level);
}

/* ================================================================= */
/*                      IOC Shell Registration                        */
/* ================================================================= */

extern "C" {

/** TmlControllerConfig — create a controller + N axis placeholders */
void TmlControllerConfig(const char *portName, const char *devicePath,
                         int numAxes, int hostId, int baudRate,
                         double movingPoll, double idlePoll)
{
    new TmlController(portName, devicePath, numAxes, hostId, baudRate,
                      movingPoll, idlePoll);
}

/** TmlAxisConfig — configure a specific axis with TML parameters */
void TmlAxisConfig(const char *portName, int axisNo,
                   int axisId, const char *setupFile,
                   const char *homingSwitch,
                   int ignoreLSP, int ignoreLSN,
                   int scrValue)
{
    TmlController *pC;
    pC = (TmlController *)findAsynPortDriver(portName);
    if (!pC) {
        printf("TmlAxisConfig: port '%s' not found\n", portName);
        return;
    }
    pC->configAxis(axisNo, axisId, setupFile, homingSwitch, ignoreLSP, ignoreLSN, scrValue);
}

/* -- TmlControllerConfig arguments -- */
static const iocshArg arg0_ctrl = {"portName",   iocshArgString};
static const iocshArg arg1_ctrl = {"devicePath", iocshArgString};
static const iocshArg arg2_ctrl = {"numAxes",    iocshArgInt};
static const iocshArg arg3_ctrl = {"hostId",     iocshArgInt};
static const iocshArg arg4_ctrl = {"baudRate",   iocshArgInt};
static const iocshArg arg5_ctrl = {"movingPoll", iocshArgDouble};
static const iocshArg arg6_ctrl = {"idlePoll",   iocshArgDouble};
static const iocshArg *const args_ctrl[] = {
    &arg0_ctrl, &arg1_ctrl, &arg2_ctrl, &arg3_ctrl,
    &arg4_ctrl, &arg5_ctrl, &arg6_ctrl
};
static const iocshFuncDef ctrlDef = {"TmlControllerConfig", 7, args_ctrl};

static void ctrlCallFunc(const iocshArgBuf *args)
{
    TmlControllerConfig(args[0].sval, args[1].sval,
                        args[2].ival, args[3].ival, args[4].ival,
                        args[5].dval, args[6].dval);
}

/* -- TmlAxisConfig arguments -- */
static const iocshArg arg0_axis = {"portName",     iocshArgString};
static const iocshArg arg1_axis = {"axisNo",       iocshArgInt};
static const iocshArg arg2_axis = {"axisId",       iocshArgInt};
static const iocshArg arg3_axis = {"setupFile",    iocshArgString};
static const iocshArg arg4_axis = {"homingSwitch", iocshArgString};
static const iocshArg arg5_axis = {"ignoreLSP", iocshArgInt};
static const iocshArg arg6_axis = {"ignoreLSN", iocshArgInt};
static const iocshArg arg7_axis = {"scrValue",  iocshArgInt};
static const iocshArg *const args_axis[] = {
    &arg0_axis, &arg1_axis, &arg2_axis, &arg3_axis, &arg4_axis,
    &arg5_axis, &arg6_axis, &arg7_axis
};
static const iocshFuncDef axisDef = {"TmlAxisConfig", 8, args_axis};

static void axisCallFunc(const iocshArgBuf *args)
{
    TmlAxisConfig(args[0].sval, args[1].ival,
                  args[2].ival, args[3].sval, args[4].sval,
                  args[5].ival, args[6].ival, args[7].ival);
}

/* -- Registrar -- */
static void TmlMotorRegister(void)
{
    iocshRegister(&ctrlDef, ctrlCallFunc);
    iocshRegister(&axisDef, axisCallFunc);
}
epicsExportRegistrar(TmlMotorRegister);

} /* extern "C" */
