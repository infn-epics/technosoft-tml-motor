/*
 * drvTmlMotor.cpp
 *
 * EPICS asynMotorController / asynMotorAxis driver for Technosoft
 * intelligent drives.
 *
 * Architecture
 * ------------
 *   TmlController  — one per RS-232/485/CAN/XPORT channel
 *   TmlAxis        — one per physical drive on the channel
 *
 * The TML communication layer is NOT thread-safe; every call is
 * serialized through tmlLock_ (an epicsMutex in the controller).
 *
 * Compile with  -DUSE_TML_NATIVE  to use the native serial protocol
 * implementation instead of the proprietary TML_lib binary library.
 *
 * Author:  Andrea Michelotti — INFN-LNF
 * Date:    2026-02
 */

#include <cstdio>
#include <cstring>
#include <cmath>
#include <unistd.h>

#include <epicsThread.h>
#include <epicsExport.h>
#include <iocsh.h>
#include <asynOctetSyncIO.h>

#ifdef USE_TML_NATIVE
#  include "tmlSerial.h"
#else
#  include <TML_lib.h>
#endif
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

/* For XPORT_IP TML_lib wants just hostname/IP (strip ":port" suffix).
 * Native mode passes the full "host:port" so connectTcp() can parse it.
 */
static void tmlDevName(const char *path, char *buf, size_t buflen)
{
    strncpy(buf, path, buflen - 1);
    buf[buflen - 1] = '\0';
#ifndef USE_TML_NATIVE
    /* TML_lib CHANNEL_XPORT_IP expects bare IP, port is implicit */
    if (tmlChannelType(path) == CHANNEL_XPORT_IP) {
        char *colon = strchr(buf, ':');
        if (colon) *colon = '\0';
    }
#endif
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

    /* Open the TML communication channel */
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
                                     const char *homingSwitch)
{
    TmlAxis *pAxis = getTmlAxis(axisNo);
    if (!pAxis) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: axis %d does not exist\n", driverName, axisNo);
        return asynError;
    }
    return pAxis->configure(axisId, setupFile, homingSwitch);
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
    , useLSP_(false)
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
                               const char *homingSwitch)
{
    if (pC_->channelFd_ < 0) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis::configure: channel not open\n");
        return asynError;
    }

    axisId_ = axisId;
    strncpy(setupFile_, setupFile, sizeof(setupFile_) - 1);

    if (homingSwitch && (strcmp(homingSwitch, "LSP") == 0 ||
                         strcmp(homingSwitch, "lsp") == 0))
        useLSP_ = true;
    else
        useLSP_ = false;

    pC_->tmlLock_.lock();

    /* Select our channel */
    TS_SelectChannel(pC_->channelFd_);

    asynStatus st = reinitAxis();
    pC_->tmlLock_.unlock();

    if (st != asynSuccess)
        return st;

    configured_ = true;
    activated_  = true;
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
    /* Load setup file (or reuse cached index — reload to be safe) */
    setupIdx_ = TS_LoadSetup(setupFile_);
    if (setupIdx_ < 0) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: reinit TS_LoadSetup('%s') FAILED: %s\n",
                  axisNo_, setupFile_, TS_GetLastErrorText());
        return asynError;
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

    DBG(1, "Axis %d reinit OK: TML-ID=%d setupIdx=%d", axisNo_, axisId_, setupIdx_);
    return asynSuccess;
}

/* ---- selectAxis helper ---- */
asynStatus TmlAxis::selectAxis()
{
    if (!configured_ || pC_->channelFd_ < 0)
        return asynError;

    if (!TS_SelectChannel(pC_->channelFd_)) {
        /* Channel may have been invalidated (e.g. XPORT TCP reconnect after
         * drive reset).  Try to reopen with the original parameters. */
        DBG(0, "TS_SelectChannel(%d) failed (%s) — attempting reconnect",
            pC_->channelFd_, TS_GetLastErrorText());
        TS_CloseChannel(pC_->channelFd_);
        char devName[256];
        tmlDevName(pC_->devicePath_, devName, sizeof(devName));
        int chType = tmlChannelType(pC_->devicePath_);
        int newFd = TS_OpenChannel(devName, (BYTE)chType,
                                   (BYTE)pC_->hostId_, (DWORD)pC_->baudRate_);
        if (newFd < 0) {
            DBG(0, "Reconnect failed: %s", TS_GetLastErrorText());
            pC_->channelFd_ = -1;
            return asynError;
        }
        pC_->channelFd_ = newFd;
        DBG(0, "Reconnected: new fd=%d — replaying axis setup", newFd);
        if (!TS_SelectChannel(pC_->channelFd_)) {
            DBG(0, "TS_SelectChannel after reconnect failed: %s", TS_GetLastErrorText());
            return asynError;
        }
        /* Replay full TML axis initialisation on the new channel */
        if (reinitAxis() != asynSuccess) {
            DBG(0, "reinitAxis after reconnect FAILED for axis %d", axisId_);
            return asynError;
        }
        /* Drive was re-initialised — power stage is off, update state */
        powered_ = false;
        setIntegerParam(pC_->tmlActive_, activated_ ? 1 : 0);
        callParamCallbacks();
        /* After reinitAxis, axis is selected — skip the TS_SelectAxis below */
        return asynSuccess;
    }

    if (!TS_SelectAxis((BYTE)axisId_)) {
        DBG(0, "TS_SelectAxis(%d) failed: %s", axisId_, TS_GetLastErrorText());
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

    /* Wait for the power stage to actually turn on (SRL bit 15) */
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
        pC_->tmlLock_.unlock();
        epicsThreadSleep(0.1);
        pC_->tmlLock_.lock();
        selectAxis();
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

    DBG(2, "Axis %d move %s pos=%.0f vel=%.2f acc=%.2f",
        axisNo_, relative ? "REL" : "ABS", position, maxVelocity, acceleration);

    pC_->tmlLock_.lock();
    asynStatus st = selectAxis();
    if (st != asynSuccess) { pC_->tmlLock_.unlock(); return st; }

    st = powerOn();
    if (st != asynSuccess) { pC_->tmlLock_.unlock(); return st; }

    homingActive_ = false;

    BOOL ok;
    long pos = (long)position;

    if (relative) {
        ok = TS_MoveRelative(pos, maxVelocity, acceleration,
                             FALSE, UPDATE_IMMEDIATE, FROM_MEASURE);
    } else {
        ok = TS_MoveAbsolute(pos, maxVelocity, acceleration,
                             UPDATE_IMMEDIATE, FROM_MEASURE);
    }

    if (ok) {
        TS_SetEventOnMotionComplete(TRUE, TRUE);
    }

    pC_->tmlLock_.unlock();

    if (!ok) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: move FAILED: %s\n",
                  axisNo_, TS_GetLastErrorText());
        return asynError;
    }

    setIntegerParam(pC_->motorStatusDone_, 0);
    setIntegerParam(pC_->motorStatusMoving_, 1);
    callParamCallbacks();

    return asynSuccess;
}

asynStatus TmlAxis::moveVelocity(double minVelocity, double maxVelocity,
                                 double acceleration)
{
    if (!configured_) return asynError;

    DBG(2, "Axis %d moveVelocity vel=%.2f acc=%.2f", axisNo_, maxVelocity, acceleration);

    pC_->tmlLock_.lock();
    asynStatus st = selectAxis();
    if (st != asynSuccess) { pC_->tmlLock_.unlock(); return st; }

    st = powerOn();
    if (st != asynSuccess) { pC_->tmlLock_.unlock(); return st; }

    homingActive_ = false;

    BOOL ok = TS_MoveVelocity(maxVelocity, acceleration,
                              UPDATE_IMMEDIATE, FROM_MEASURE);
    pC_->tmlLock_.unlock();

    if (!ok) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: moveVelocity FAILED: %s\n",
                  axisNo_, TS_GetLastErrorText());
        return asynError;
    }

    setIntegerParam(pC_->motorStatusDone_, 0);
    setIntegerParam(pC_->motorStatusMoving_, 1);
    callParamCallbacks();

    return asynSuccess;
}

asynStatus TmlAxis::home(double minVelocity, double maxVelocity,
                         double acceleration, int forwards)
{
    if (!configured_) return asynError;

    DBG(2, "Axis %d home forwards=%d vel=%.2f", axisNo_, forwards, maxVelocity);

    pC_->tmlLock_.lock();
    asynStatus st = selectAxis();
    if (st != asynSuccess) { pC_->tmlLock_.unlock(); return st; }

    st = powerOn();
    if (st != asynSuccess) { pC_->tmlLock_.unlock(); return st; }

    /*
     * Homing strategy (identical to the original NDS driver):
     *   1. Start a velocity move towards the chosen limit switch (LSP/LSN).
     *   2. Set an event on limit switch.
     *   3. The poller will detect motion-complete when the limit switch fires.
     *   4. Then it sets the position to 0.
     */
    double speed = forwards ? fabs(maxVelocity) : -fabs(maxVelocity);

    /* Choose LS direction based on useLSP_ flag */
    if (useLSP_) {
        speed = fabs(maxVelocity);
    } else {
        speed = -fabs(maxVelocity);
    }

    BOOL ok = TS_MoveVelocity(speed, fabs(acceleration),
                              UPDATE_IMMEDIATE, FROM_MEASURE);
    if (ok) {
        short lsType = useLSP_ ? LSW_POSITIVE : LSW_NEGATIVE;
        short transition = useLSP_ ? TRANSITION_LOW_TO_HIGH : TRANSITION_LOW_TO_HIGH;
        ok = TS_SetEventOnLimitSwitch(lsType, transition, TRUE, TRUE);
    }

    pC_->tmlLock_.unlock();

    if (!ok) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: home FAILED: %s\n",
                  axisNo_, TS_GetLastErrorText());
        return asynError;
    }

    homingActive_ = true;
    setIntegerParam(pC_->motorStatusDone_, 0);
    setIntegerParam(pC_->motorStatusMoving_, 1);
    setIntegerParam(pC_->motorStatusHomed_, 0);
    callParamCallbacks();

    return asynSuccess;
}

asynStatus TmlAxis::stop(double acceleration)
{
    if (!configured_) return asynError;

    DBG(2, "Axis %d stop", axisNo_);

    pC_->tmlLock_.lock();
    selectAxis();

    /* First try to abort any TML subroutine in progress */
    TS_ABORT();
    /* Then stop the motion */
    TS_Stop();

    pC_->tmlLock_.unlock();

    homingActive_ = false;
    setIntegerParam(pC_->motorStatusDone_, 1);
    setIntegerParam(pC_->motorStatusMoving_, 0);
    callParamCallbacks();

    return asynSuccess;
}

asynStatus TmlAxis::setPosition(double position)
{
    if (!configured_) return asynError;

    DBG(2, "Axis %d setPosition %.0f", axisNo_, position);

    pC_->tmlLock_.lock();
    asynStatus st = selectAxis();
    if (st != asynSuccess) { pC_->tmlLock_.unlock(); return st; }

    BOOL ok = TS_SetPosition((long)position);
    pC_->tmlLock_.unlock();

    if (!ok) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: setPosition FAILED: %s\n",
                  axisNo_, TS_GetLastErrorText());
        return asynError;
    }

    return asynSuccess;
}

/* ---- setClosedLoop: map to TML power stage ON/OFF ---- */
asynStatus TmlAxis::setClosedLoop(bool closedLoop)
{
    if (!configured_) return asynError;

    DBG(2, "Axis %d setClosedLoop %s", axisNo_, closedLoop ? "ON" : "OFF");

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

    /* ---- Read position ---- */
    long pos = 0;
    BOOL posOk = TS_GetLongVariable("TPOS", pos);

    /* ---- Read actual encoder position (APOS) ---- */
    long apos = 0;
    BOOL aposOk = TS_GetLongVariable("APOS", apos);

    /* ---- Read commanded speed (CSPD) as 32-bit fixed 16.16 ---- */
    long cspd_raw = 0;
    TS_GetLongVariable("CSPD", cspd_raw);

    /* ---- Read additional status registers: MCR, MSR, ISR ---- */
    WORD mcr_w = 0, msr_w = 0, isr_w = 0;
    TS_ReadStatus(REG_MCR, mcr_w);
    TS_ReadStatus(REG_MSR, msr_w);
    TS_ReadStatus(REG_ISR, isr_w);

    /* ---- Read status registers ---- */
    unsigned short srh, srl, mer;
    asynStatus regSt = readRegisters(srh, srl, mer);

    pC_->tmlLock_.unlock();

    if (!posOk || regSt != asynSuccess) {
        setIntegerParam(pC_->motorStatusCommsError_, 1);
        callParamCallbacks();
        return asynError;
    }

    /* Position */
    setDoubleParam(pC_->motorPosition_, (double)pos);

    /* Encoder position (APOS) — set both motorEncoderPosition and TML_APOS */
    if (aposOk) {
        setDoubleParam(pC_->motorEncoderPosition_, (double)apos);
        setDoubleParam(pC_->tmlAPOS_, (double)apos);
    }

    /* Commanded speed (CSPD): fixed-point 16.16 → double */
    double cspd_val = (double)cspd_raw / 65536.0;
    setDoubleParam(pC_->tmlCSPD_, cspd_val);

    /* Publish raw registers */
    setIntegerParam(pC_->tmlSRH_, (int)srh);
    setIntegerParam(pC_->tmlSRL_, (int)srl);
    setIntegerParam(pC_->tmlMER_, (int)mer);
    setIntegerParam(pC_->tmlMCR_, (int)mcr_w);
    setIntegerParam(pC_->tmlMSR_, (int)msr_w);
    setIntegerParam(pC_->tmlISR_, (int)isr_w);

    /* ---- Map TML status to motor record status bits ---- */

    /* Motion complete = SRL bit 10 */
    bool motionComplete = (srl & SRL_BIT_MOTION_COMPLETE) != 0;
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

    /* Sync powered_ / activated_ with hardware-reported AxisON state */
    if (axisON && !powered_) {
        powered_   = true;
        activated_ = true;
    } else if (!axisON && powered_) {
        powered_ = false;
    }

    setIntegerParam(pC_->motorStatusDone_,      motionComplete ? 1 : 0);
    setIntegerParam(pC_->motorStatusMoving_,     *moving ? 1 : 0);
    setIntegerParam(pC_->motorStatusHighLimit_,  lsp ? 1 : 0);
    setIntegerParam(pC_->motorStatusLowLimit_,   lsn ? 1 : 0);
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

    /* Homing complete logic: if we were homing and motion is now complete */
    if (homingActive_ && motionComplete) {
        /* Set position to zero at home */
        pC_->tmlLock_.lock();
        selectAxis();
        TS_SetPosition(0);
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
                   const char *homingSwitch)
{
    TmlController *pC;
    pC = (TmlController *)findAsynPortDriver(portName);
    if (!pC) {
        printf("TmlAxisConfig: port '%s' not found\n", portName);
        return;
    }
    pC->configAxis(axisNo, axisId, setupFile, homingSwitch);
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
static const iocshArg *const args_axis[] = {
    &arg0_axis, &arg1_axis, &arg2_axis, &arg3_axis, &arg4_axis
};
static const iocshFuncDef axisDef = {"TmlAxisConfig", 5, args_axis};

static void axisCallFunc(const iocshArgBuf *args)
{
    TmlAxisConfig(args[0].sval, args[1].ival,
                  args[2].ival, args[3].sval, args[4].sval);
}

/* -- Registrar -- */
static void TmlMotorRegister(void)
{
    iocshRegister(&ctrlDef, ctrlCallFunc);
    iocshRegister(&axisDef, axisCallFunc);
}
epicsExportRegistrar(TmlMotorRegister);

} /* extern "C" */
