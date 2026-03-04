TOP = ../..
include $(TOP)/configure/CONFIG

# -------------------------------------------------------
# IOC binary — links against libTechnosoftMotor from technosoftSup
# -------------------------------------------------------
PROD_IOC = technosoft

DBD += technosoft.dbd
technosoft_DBD += base.dbd
technosoft_DBD += asyn.dbd
technosoft_DBD += motorSupport.dbd
technosoft_DBD += devTechnosoft.dbd

# -------------------------------------------------------
# Build mode selection (must match technosoftSup)
# -------------------------------------------------------
USE_TML_NATIVE ?= YES

ifeq ($(USE_TML_NATIVE),YES)
USR_CPPFLAGS += -DUSE_TML_NATIVE
else
# TML library paths (needed at link time for TML_lib mode)
USR_LDFLAGS += -L$(MOT_TML_LIB)/lib
endif

# -------------------------------------------------------
# Sources — only the IOC main and register glue
# -------------------------------------------------------
technosoft_SRCS += technosoft_registerRecordDeviceDriver.cpp
technosoft_SRCS_DEFAULT += technosoftMain.cpp

# -------------------------------------------------------
# Libraries
# -------------------------------------------------------
technosoft_LIBS += TechnosoftMotor
technosoft_LIBS += motor
technosoft_LIBS += asyn
technosoft_LIBS += $(EPICS_BASE_IOC_LIBS)

ifneq ($(USE_TML_NATIVE),YES)
technosoft_SYS_LIBS += TML_lib
technosoft_SYS_LIBS += tmlcomm
technosoft_SYS_LIBS += z
endif

include $(TOP)/configure/RULES
