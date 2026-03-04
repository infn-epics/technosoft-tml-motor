#!../../bin/linux-x86_64/technosoft
#
# st.cmd — Technosoft TML Motor IOC startup script
#

< envPaths

## Register all support components
dbLoadDatabase "${TOP}/dbd/technosoft.dbd"
technosoft_registerRecordDeviceDriver pdbbase

## Load motor utility database (optional)
# dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=TML:,RECS=5,TIMEOUT=10")

## -------------------------------------------------------
## Choose one of the example command files below, or write
## your own with TmlControllerConfig + TmlAxisConfig calls.
## -------------------------------------------------------

# Single axis example (RS-232, /dev/ttyUSB0):
#< singleAxis.cmd

# Multi-axis example:
# < multiAxis.cmd

# SPARC TML channel 1 (10 axes via Moxa NPort @ scsparcmoxa001.lnf.infn.it:4001)
< tml-ch1.cmd

cd "${TOP}/iocBoot/${IOC}"
var drvTmlDebug 1

iocInit

## Start motor record utility (optional)
# motorUtilInit("TML:")
#dbl
