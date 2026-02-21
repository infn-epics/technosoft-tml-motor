/*
 * drvTmlMotor.h
 *
 * EPICS asynMotorController / asynMotorAxis driver for Technosoft
 * intelligent drives.
 *
 * Compile with  -DUSE_TML_NATIVE  to use the native serial protocol
 * implementation (tmlSerial) instead of the proprietary TML_lib binary.
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

/* Forward declaration */
class TmlAxis;

/* ---------- Extra asyn parameters exposed by this driver ---------- */
#define TML_SRH_String        "TML_SRH"        /* asynInt32, R   */
#define TML_SRL_String        "TML_SRL"        /* asynInt32, R   */
#define TML_MER_String        "TML_MER"        /* asynInt32, R   */
#define TML_SETUP_FILE_String "TML_SETUP_FILE" /* asynOctet, R/W (per-axis) */

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
                          const char *homingSwitch);

    /* Locking around all TML_lib calls (the library is NOT thread-safe) */
    epicsMutex &tmlLock() { return tmlLock_; }

    int channelFd()  const { return channelFd_; }
    int hostId()     const { return hostId_; }

    /* Report */
    void report(FILE *fp, int level) override;

protected:
    int channelFd_;        /* TML channel file descriptor */
    int hostId_;
    int baudRate_;
    char devicePath_[256];
    epicsMutex tmlLock_;

    /* Extra parameter indices */
    int tmlSRH_;
    int tmlSRL_;
    int tmlMER_;
    int tmlSetupFile_;

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
     */
    asynStatus configure(int axisId, const char *setupFile, const char *homingSwitch);

    /* ---- asynMotorAxis interface ---- */
    asynStatus move(double position, int relative, double minVelocity,
                    double maxVelocity, double acceleration) override;
    asynStatus moveVelocity(double minVelocity, double maxVelocity,
                            double acceleration) override;
    asynStatus home(double minVelocity, double maxVelocity,
                    double acceleration, int forwards) override;
    asynStatus stop(double acceleration) override;
    asynStatus setPosition(double position) override;
    asynStatus poll(bool *moving) override;

    void report(FILE *fp, int level) override;

private:
    TmlController *pC_;

    int  axisId_;              /* TML drive address (1-255) */
    int  setupIdx_;            /* Index returned by TS_LoadSetup */
    bool configured_;          /* True after successful configure() */
    bool powered_;             /* Current power-stage state */
    bool homingActive_;        /* True while homing in progress */
    bool useLSP_;              /* true = home on LSP, false = LSN */

    char setupFile_[512];

    /* Helper: select this axis's channel + axis before every TML call */
    asynStatus selectAxis();

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
                       const char *homingSwitch);
}

#endif /* DRV_TML_MOTOR_H */
