#!../../bin/linux-x86_64/technosoft
#
# st.cmd — Technosoft TML Motor IOC startup script
#

< envPaths

## Register all support components
cd "${TOP}"
dbLoadDatabase "dbd/technosoft.dbd"
technosoft_registerRecordDeviceDriver pdbbase

## Load motor utility database (optional)
# dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=TML:,RECS=5,TIMEOUT=10")

## -------------------------------------------------------
## Choose one of the example command files below, or write
## your own with TmlControllerConfig + TmlAxisConfig calls.
## -------------------------------------------------------

# Single axis example (RS-232, /dev/ttyUSB0):
< singleAxis.cmd

# Multi-axis example:
# < multiAxis.cmd

cd "${TOP}/iocBoot/${IOC}"
iocInit

## Start motor record utility (optional)
# motorUtilInit("TML:")
