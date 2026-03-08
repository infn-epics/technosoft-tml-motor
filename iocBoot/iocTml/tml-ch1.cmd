#
# tml-ch1.cmd — SPARC TML multi-axis motor channel 1
#
# Moxa NPort 5250A at scsparcmoxa001.lnf.infn.it (192.168.190.55)
# RS-232 serial port 1  →  TCP port 4001
#

# --- Load motor record database ---
dbLoadTemplate("tml-ch1.substitutions")

# --- Controller ---
#   TmlControllerConfig(portName, devicePath, numAxes, hostId, baudRate, movingPoll, idlePoll)
#
#   hostId = 5 — TML master node ID for this bus.
# TmlControllerConfig("tml-ch1", "/var/tmp/ttyV0", 10, 5, 9600, 0.1, 1.0)
TmlControllerConfig("tml-ch1", "scsparcmoxa001.lnf.infn.it:4001", 10, 5, 9600, 0.1, 1.0)
# --- Axis configuration ---
#   TmlAxisConfig(portName, axisNo, axisId, setupFile, homingSwitch, ignoreLSP, ignoreLSN, scrValue)
#
#   ignoreLSP: 1 = suppress HLS (positive limit) when input is not connected
#   ignoreLSN: 1 = suppress LLS (negative limit) when input is not connected
#   Set to 0 if the limit switch is actually wired.
#   scrValue:  SCR register value to enable encoder feedback (e.g. 0x4338).
#              Set to 0 to skip (use setup file defaults).
#
#   NOTE: Replace the .t.zip paths below with the actual setup files
#         exported from Technosoft EasyMotion/EasySetup for each drive.

# Ax 0  GUNFLG01  (axis ID 10)
TmlAxisConfig("tml-ch1", 0, 10, "$(TOP)/tml_lib/config/nanotec-st5709s1208.t.zip", "LSN", 0, 0)

# Ax 1  GUNSOLV1  (axis ID 7)
TmlAxisConfig("tml-ch1", 1,  7, "$(TOP)/tml_lib/config/nanotec-st5709s1208.t.zip", "LSN", 0, 0)

# Ax 2  GUNSOLH1  (axis ID 8)
TmlAxisConfig("tml-ch1", 2,  8, "$(TOP)/tml_lib/config/nanotec-st5709s1208.t.zip", "LSN", 0, 0)

# Ax 3  AC1FLG01  (axis ID 3)
TmlAxisConfig("tml-ch1", 3,  3, "$(TOP)/tml_lib/config/nanotec-st5709s1208.t.zip", "LSN", 0, 0)

# Ax 4  AC2FLG01  (axis ID 4)
TmlAxisConfig("tml-ch1", 4,  4, "$(TOP)/tml_lib/config/nanotec-st5709s1208.t.zip", "LSN", 0, 0)

# Ax 5  AC3FLG01  (axis ID 5)
TmlAxisConfig("tml-ch1", 5,  5, "$(TOP)/tml_lib/config/nanotec-st5709s1208.t.zip", "LSN", 0, 0)

# Ax 6  UTLFLG01  (axis ID 12)
TmlAxisConfig("tml-ch1", 6, 12, "$(TOP)/tml_lib/config/nanotec-st5709s1208.t.zip", "LSN", 0, 0)

# Ax 7  UTLFLG02  (axis ID 13)
TmlAxisConfig("tml-ch1", 7, 13, "$(TOP)/tml_lib/config/nanotec-st5709s1208.t.zip", "LSN", 0, 0)

# Ax 8  SBNFLG00  (axis ID 11)
TmlAxisConfig("tml-ch1", 8, 11, "$(TOP)/tml_lib/config/nanotec-st5709s1208.t.zip", "LSN", 0, 0)

# Ax 9  SBNFLG01  (axis ID 14)
TmlAxisConfig("tml-ch1", 9, 14, "$(TOP)/tml_lib/config/nanotec-st5709s1208.t.zip", "LSN", 0, 0)
