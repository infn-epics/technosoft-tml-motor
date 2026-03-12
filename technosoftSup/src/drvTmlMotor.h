/*
 * drvTmlMotor.h
 *
 * EPICS asynMotorController / asynMotorAxis driver for Technosoft
 * intelligent drives.  Uses the native serial TML protocol (tmlSerial).
 *
 * Author:  Andrea Michelotti — INFN-LNF
 * Date:    2026-02
 */

#ifndef DRV_TML_MOTOR_H
#define DRV_TML_MOTOR_H

#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsTime.h>

/* Forward declaration */
class TmlAxis;

/* ---------- Extra asyn parameters exposed by this driver ---------- */
#define TML_SRH_String          "TML_SRH"          /* asynInt32, R   */
#define TML_SRL_String          "TML_SRL"          /* asynInt32, R   */
#define TML_MER_String          "TML_MER"          /* asynInt32, R   */
#define TML_MCR_String          "TML_MCR"          /* asynInt32, R   */
#define TML_MSR_String          "TML_MSR"          /* asynInt32, R   */
#define TML_ISR_String          "TML_ISR"          /* asynInt32, R   */
#define TML_SETUP_FILE_String   "TML_SETUP_FILE"   /* asynOctet, R/W (per-axis) */
#define TML_ACTIVE_String       "TML_ACTIVE"       /* asynInt32, R   — 1=axis initialized+powered, 0=inactive */
#define TML_FAULT_TEXT_String   "TML_FAULT_TEXT"    /* asynOctet, R   — human-readable fault string */
#define TML_APOS_String         "TML_APOS"         /* asynFloat64, R — actual encoder position */
#define TML_CSPD_String         "TML_CSPD"         /* asynFloat64, R — commanded speed readback */
#define TML_RESET_FAULT_String  "TML_RESET_FAULT"  /* asynInt32, W   — write 1 to reset faults */
#define TML_SAVE_EEPROM_String  "TML_SAVE_EEPROM"  /* asynInt32, W   — write 1 to save to EEPROM */
#define TML_RESET_DRIVE_String  "TML_RESET_DRIVE"  /* asynInt32, W   — write 1 to reset drive */
#define TML_POTM_String         "TML_POTM"         /* asynFloat64, R — potentiometer / ADC readback */
#define TML_FORCE_HOME_String   "TML_FORCE_HOME"   /* asynInt32, W   — write 1 to force-home (set pos=0 + homed) */

#define NUM_TML_PARAMS 16

/* ================================================================= */
/*                         TmlController                              */
/* ================================================================= */

class TmlController : public asynMotorController
{
public:
    /**
     * @param portName   EPICS asyn port name
     * @param devicePath Serial device path, e.g. "/dev/ttyUSB0" or IP:port for XPORT
     * @param numAxes    Number of axes on this communication channel
     * @param hostId     Host (PC) address on the RS-232/485/CAN bus (1-255)
     * @param baudRate   Serial baud rate (default 9600)
     * @param movingPoll Polling period (s) while any axis is moving
     * @param idlePoll   Polling period (s) while all axes are idle
     */
    TmlController(const char *portName, const char *devicePath,
                  int numAxes, int hostId, int baudRate,
                  double movingPoll, double idlePoll);

    virtual ~TmlController();

    /* Convenience: return typed pointer (non-virtual) */
    TmlAxis *getTmlAxis(int axisNo);
    TmlAxis *getTmlAxis(asynUser *pasynUser);

    /* Called from iocsh to configure a single axis after controller creation */
    asynStatus configAxis(int axisNo, int axisId, const char *setupFile,
                          const char *homingSwitch,
                          int ignoreLSP = 0, int ignoreLSN = 0,
                          int scrValue = 0);

    /* Write handler for command parameters (reset fault, save, etc.) */
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value) override;

    /* Locking around all TML calls (the native library is NOT thread-safe) */
    epicsMutex &tmlLock() { return tmlLock_; }

    int channelFd()  const { return channelFd_; }
    int hostId()     const { return hostId_; }

    /* Report */
    void report(FILE *fp, int level) override;

protected:
    int channelFd_;        /* TML channel file descriptor (shared by all axes) */
    int hostId_;
    int baudRate_;
    char devicePath_[256];
    epicsMutex tmlLock_;
    epicsTimeStamp lastReconnect_;   /* rate-limit reconnects */

    /* Extra parameter indices */
    int tmlSRH_;
    int tmlSRL_;
    int tmlMER_;
    int tmlMCR_;
    int tmlMSR_;
    int tmlISR_;
    int tmlSetupFile_;
    int tmlActive_;
    int tmlFaultText_;
    int tmlAPOS_;
    int tmlCSPD_;
    int tmlResetFault_;
    int tmlSaveEeprom_;
    int tmlResetDrive_;
    int tmlPOTM_;
    int tmlForceHome_;

    friend class TmlAxis;
};

/* ================================================================= */
/*                            TmlAxis                                 */
/* ================================================================= */

class TmlAxis : public asynMotorAxis
{
public:
    TmlAxis(TmlController *pC, int axisNo);
    virtual ~TmlAxis();

    /**
     * Configure this axis with TML parameters.
     * Must be called before iocInit().
     * @param axisId        TML axis ID (1-255)
     * @param setupFile     Path to .t.zip or setup directory
     * @param homingSwitch  "LSP" or "LSN"
     * @param ignoreLSP  If true, do not report HLS (positive limit) to motor record
     * @param ignoreLSN  If true, do not report LLS (negative limit) to motor record
     */
    asynStatus configure(int axisId, const char *setupFile, const char *homingSwitch,
                         bool ignoreLSP = false, bool ignoreLSN = false,
                         int scrValue = 0);

    /* ---- asynMotorAxis interface ---- */
    asynStatus move(double position, int relative, double minVelocity,
                    double maxVelocity, double acceleration) override;
    asynStatus moveVelocity(double minVelocity, double maxVelocity,
                            double acceleration) override;
    asynStatus home(double minVelocity, double maxVelocity,
                    double acceleration, int forwards) override;
    asynStatus stop(double acceleration) override;
    asynStatus setPosition(double position) override;
    asynStatus setClosedLoop(bool closedLoop) override;
    asynStatus poll(bool *moving) override;

    void report(FILE *fp, int level) override;

    /* Helper: select this axis's channel + axis before every TML call */
    asynStatus selectAxis();

    /* Force-home: set position to 0 and mark homed without motion */
    asynStatus forceHome();

private:
    TmlController *pC_;

    int  axisId_;              /* TML drive address (1-255) */
    int  setupIdx_;            /* Index returned by TS_LoadSetup */
    bool configured_;          /* True after successful configure() */
    bool activated_;           /* True after DriveInitialisation + Power ON */
    bool powered_;             /* Current power-stage state */
    bool homingActive_;        /* True while homing in progress */
    bool homingMoveSeen_;      /* True after at least one poll saw MC=0 (drive moved) */
    bool homingTowardLSP_;     /* True if current home is toward LSP, false = LSN */
    bool stopping_;            /* True after stop() until hardware confirms MC */
    bool useLSP_;              /* true = home on LSP, false = LSN */
    bool ignoreLSP_;           /* true = do not report HLS (positive limit unconnected) */
    bool ignoreLSN_;           /* true = do not report LLS (negative limit unconnected) */
    int  scrValue_;            /* SCR register value to write after init (0 = skip) */
    bool needsReinit_;         /* True if init failed, retry in poll */
    int  reinitCountdown_;     /* Poll cycles to wait before next retry */
    int  reinitBackoff_;       /* Current backoff multiplier (doubles each fail) */
    int  pollCount_;           /* Per-axis poll counter for throttled reads */

    char setupFile_[512];

    /* Replay LoadSetup+SetupAxis+SelectAxis+DriveInitialisation on current channel.
     * Called both from configure() and from selectAxis() after a channel reconnect.
     * Must be called with tmlLock_ held. */
    asynStatus reinitAxis();

    /* Read SRH, SRL, MER registers and set motor-record status bits */
    asynStatus readRegisters(unsigned short &srh, unsigned short &srl, unsigned short &mer);

    /* Ensure power stage is ON */
    asynStatus powerOn();
    asynStatus powerOff();
};

/* IOC-shell registration */
extern "C" {
    void TmlControllerConfig(const char *portName, const char *devicePath,
                             int numAxes, int hostId, int baudRate,
                             double movingPoll, double idlePoll);
    void TmlAxisConfig(const char *portName, int axisNo,
                       int axisId, const char *setupFile,
                       const char *homingSwitch,
                       int ignoreLSP, int ignoreLSN,
                       int scrValue);
}

#endif /* DRV_TML_MOTOR_H */
