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
#   devicePath — OPTION A: "IP"          direct XPORT/Ethernet (CHANNEL_XPORT_IP, TML_lib auto-manages port)
#              — OPTION B: "IP:port"     direct TCP, native serial protocol (USE_TML_NATIVE=YES only)
#              — OPTION C: /dev/ttyUSBx  local serial RS-232/RS-485
#              — OPTION D: /var/tmp/ttyV0  socat PTY bridge (see tools/start_sim.sh)
#   numAxes    — total axes on this channel
#   hostId     — RS-232/485 host ID; for XPORT usually the axis ID of the direct drive
#   baudRate   — 9600 (default) or 115200; for XPORT_IP TML_lib ignores this
#   movingPoll — polling period (s) while moving
#   idlePoll   — polling period (s) while idle

# Option A: direct XPORT connection (recommended for hardware with Ethernet XPORT)
# TmlControllerConfig("TML1", "/var/tmp/ttyV0", 1, 15, 9600, 0.1, 1.0)
# socat pty,link=/var/tmp/ttyV0,raw,echo=0,b9600 tcp:192.168.190.55:4001 

# Option B: TCP with native protocol (USE_TML_NATIVE=YES build)
# make USE_TML_NATIVE=YES 
TmlControllerConfig("TML1", "192.168.190.55:4001", 1, 15, 9600, 0.1, 1.0)

# Option C/D: local serial or socat PTY
#TmlControllerConfig("TML1", "/var/tmp/ttyV0", 1, 15, 9600, 0.1, 1.0)
#TmlControllerConfig("TML1", "/dev/ttyUSB0",   1, 15, 9600, 0.1, 1.0)


## Configure axis 0
#   TmlAxisConfig(portName, axisNo, axisId, setupFile, homingSwitch, ignoreLSP, ignoreLSN, scrValue)
#
#   portName     — must match the controller's portName
#   axisNo       — 0-based index into motor record array
#   axisId       — TML drive address on the bus (1-255)
#   setupFile    — path to .t.zip or setup directory from EasyMotion/EasySetup
#   homingSwitch — "LSN" or "LSP" (which limit switch to use for homing)
#   scrValue     — SCR register value (hex) to enable encoder; 0 = skip
#                  e.g. 0x4338 for open-loop with encoder feedback
# 17208
TmlAxisConfig("TML1", 0, 15, "$(TOP)/tml_lib/config/star_vat_phs.t.zip", "LSN",0,0,17208)
