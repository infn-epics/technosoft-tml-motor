/*
 * drvTmlMotor.cpp
 *
 * EPICS asynMotorController / asynMotorAxis driver for Technosoft
 * intelligent drives using the TML_lib high-level library.
 *
 * Architecture
 * ------------
 *   TmlController  — one per RS-232/485/CAN/XPORT channel
 *   TmlAxis        — one per physical drive on the channel
 *
 * The TML_lib library is NOT thread-safe; every call is serialized
 * through tmlLock_ (an epicsMutex in the controller).
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

#include <TML_lib.h>
#include "drvTmlMotor.h"

/* Global debug level — settable from iocsh via "var drvTmlDebug N" */
int drvTmlDebug = 0;
extern "C" { epicsExportAddress(int, drvTmlDebug); }

#define DBG(level, fmt, ...) \
    do { if (drvTmlDebug >= (level)) \
        printf("drvTml [%s:%d] " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); \
    } while(0)

/* SRL register bits */
#define SRL_BIT_MOTION_COMPLETE  (1 << 10)
#define SRL_BIT_AXIS_ON          (1 << 15)

/* SRH register bits */
#define SRH_BIT_FAULT            (1 << 15)

/* MER register bits */
#define MER_BIT_LSP              (1 << 6)
#define MER_BIT_LSN              (1 << 7)

/* ================================================================= */
/*                       TmlController                                */
/* ================================================================= */

static const char *driverName = "TmlController";

TmlController::TmlController(const char *portName, const char *devicePath,
                             int numAxes, int hostId, int baudRate,
                             double movingPoll, double idlePoll)
    : asynMotorController(portName, numAxes,
                          4,  /* extra asyn params: SRH, SRL, MER, SETUP_FILE */
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
    createParam(TML_SRH_String,        asynParamInt32,  &tmlSRH_);
    createParam(TML_SRL_String,        asynParamInt32,  &tmlSRL_);
    createParam(TML_MER_String,        asynParamInt32,  &tmlMER_);
    createParam(TML_SETUP_FILE_String, asynParamOctet,  &tmlSetupFile_);

    /* Open the TML communication channel */
    DBG(1, "Opening channel '%s' hostId=%d baud=%d", devicePath_, hostId_, baudRate_);

    tmlLock_.lock();
    channelFd_ = TS_OpenChannel(devicePath_, CHANNEL_RS232, (BYTE)hostId_, (DWORD)baudRate_);
    tmlLock_.unlock();

    if (channelFd_ < 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: TS_OpenChannel('%s') FAILED: %s\n",
                  driverName, devicePath_, TS_GetLastErrorText());
    } else {
        DBG(1, "Channel opened, fd=%d", channelFd_);
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

/* ================================================================= */
/*                            TmlAxis                                 */
/* ================================================================= */

TmlAxis::TmlAxis(TmlController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo)
    , pC_(pC)
    , axisId_(0)
    , setupIdx_(-1)
    , configured_(false)
    , powered_(false)
    , homingActive_(false)
    , useLSP_(false)
{
    memset(setupFile_, 0, sizeof(setupFile_));
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

    /* Load setup (returns index >= 0 or -1 on error) */
    setupIdx_ = TS_LoadSetup(setupFile_);
    if (setupIdx_ < 0) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: TS_LoadSetup('%s') FAILED: %s\n",
                  axisNo_, setupFile_, TS_GetLastErrorText());
        pC_->tmlLock_.unlock();
        return asynError;
    }

    /* Setup and select the axis */
    if (!TS_SetupAxis((BYTE)axisId_, setupIdx_)) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: TS_SetupAxis(%d,%d) FAILED: %s\n",
                  axisNo_, axisId_, setupIdx_, TS_GetLastErrorText());
        pC_->tmlLock_.unlock();
        return asynError;
    }

    if (!TS_SelectAxis((BYTE)axisId_)) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: TS_SelectAxis(%d) FAILED: %s\n",
                  axisNo_, axisId_, TS_GetLastErrorText());
        pC_->tmlLock_.unlock();
        return asynError;
    }

    /* Drive initialisation (download setup to drive) */
    if (!TS_DriveInitialisation()) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "TmlAxis[%d]: TS_DriveInitialisation FAILED: %s\n",
                  axisNo_, TS_GetLastErrorText());
        pC_->tmlLock_.unlock();
        return asynError;
    }

    pC_->tmlLock_.unlock();

    configured_ = true;

    /* Publish setup file as asyn parameter */
    pC_->setStringParam(axisNo_, pC_->tmlSetupFile_, setupFile_);

    DBG(1, "Axis %d configured: TML-ID=%d setup='%s' homing=%s",
        axisNo_, axisId_, setupFile_, useLSP_ ? "LSP" : "LSN");

    return asynSuccess;
}

/* ---- selectAxis helper ---- */
asynStatus TmlAxis::selectAxis()
{
    if (!configured_ || pC_->channelFd_ < 0)
        return asynError;
    TS_SelectChannel(pC_->channelFd_);
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
        setIntegerParam(pC_->motorStatusCommsError_, 1);
        callParamCallbacks();
        return asynError;
    }

    /* ---- Read position ---- */
    long pos = 0;
    BOOL posOk = TS_GetLongVariable("TPOS", pos);

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

    /* Publish raw registers */
    setIntegerParam(pC_->tmlSRH_, (int)srh);
    setIntegerParam(pC_->tmlSRL_, (int)srl);
    setIntegerParam(pC_->tmlMER_, (int)mer);

    /* ---- Map TML status to motor record status bits ---- */

    /* Motion complete = SRL bit 10 */
    bool motionComplete = (srl & SRL_BIT_MOTION_COMPLETE) != 0;
    bool axisON         = (srl & SRL_BIT_AXIS_ON) != 0;
    bool fault          = (srh & SRH_BIT_FAULT) != 0;
    bool lsp            = (mer & MER_BIT_LSP) != 0;
    bool lsn            = (mer & MER_BIT_LSN) != 0;

    *moving = !motionComplete && axisON;

    setIntegerParam(pC_->motorStatusDone_,      motionComplete ? 1 : 0);
    setIntegerParam(pC_->motorStatusMoving_,     *moving ? 1 : 0);
    setIntegerParam(pC_->motorStatusHighLimit_,  lsp ? 1 : 0);
    setIntegerParam(pC_->motorStatusLowLimit_,   lsn ? 1 : 0);
    setIntegerParam(pC_->motorStatusPowerOn_,    axisON ? 1 : 0);
    setIntegerParam(pC_->motorStatusProblem_,    fault ? 1 : 0);
    setIntegerParam(pC_->motorStatusCommsError_, 0);

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
    fprintf(fp, "  Axis %d: TML-ID=%d configured=%s setup='%s' powered=%s homing=%s home=%s\n",
            axisNo_, axisId_,
            configured_ ? "yes" : "no",
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
