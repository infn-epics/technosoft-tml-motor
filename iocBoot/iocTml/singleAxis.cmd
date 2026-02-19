#
# singleAxis.cmd — Example: single Technosoft axis on RS-232
#
# Usage: edit singleAxis.substitutions to set PV names, resolution, etc.
#        then include this file from st.cmd:  < singleAxis.cmd
#

# Load the motor record database
dbLoadTemplate("singleAxis.substitutions")

## Create the controller
#   TmlControllerConfig(portName, devicePath, numAxes, hostId, baudRate, movingPoll, idlePoll)
#
#   portName   — EPICS asyn port name (must match PORT in substitutions)
#   devicePath — /dev/ttyUSBx for local serial, or IP:port for XPORT
#   numAxes    — total axes on this channel
#   hostId     — RS-232 host ID (usually the axis ID of direct-connected drive)
#   baudRate   — 9600 (default) or 115200
#   movingPoll — polling period (s) while moving
#   idlePoll   — polling period (s) while idle

TmlControllerConfig("TML1", "/dev/ttyUSB0", 1, 255, 9600, 0.1, 1.0)

## Configure axis 0
#   TmlAxisConfig(portName, axisNo, axisId, setupFile, homingSwitch)
#
#   portName     — must match the controller's portName
#   axisNo       — 0-based index into motor record array
#   axisId       — TML drive address on the bus (1-255)
#   setupFile    — path to .t.zip or setup directory from EasyMotion/EasySetup
#   homingSwitch — "LSN" or "LSP" (which limit switch to use for homing)

TmlAxisConfig("TML1", 0, 1, "$(TOP)/tml_lib/config/nanotec-st5709s1208.t.zip", "LSN")
