TOP = ../..
include $(TOP)/configure/CONFIG

# -------------------------------------------------------
# Support library: libTechnosoftMotor.so
# -------------------------------------------------------
LIBRARY_IOC = TechnosoftMotor

DBD += devTechnosoft.dbd

# -------------------------------------------------------
# Build mode selection:
#   USE_TML_NATIVE=YES  → native serial protocol (no TML_lib)
#   USE_TML_NATIVE=NO   → original TML_lib binary (default)
#
# Set in configure/CONFIG_SITE or on the command line:
#   make USE_TML_NATIVE=YES
# -------------------------------------------------------
USE_TML_NATIVE ?= YES

ifeq ($(USE_TML_NATIVE),YES)

# ---- Native serial TML protocol ----
USR_CPPFLAGS += -DUSE_TML_NATIVE
TechnosoftMotor_SRCS += tmlSerial.cpp

else

# ---- Original TML_lib binary library ----
USR_CPPFLAGS += -I$(MOT_TML_LIB)/include
USR_LDFLAGS  += -L$(MOT_TML_LIB)/lib

# Install vendor shared libraries
LIB_INSTALLS += $(MOT_TML_LIB)/lib/libTML_lib.so
LIB_INSTALLS += $(MOT_TML_LIB)/lib/libtmlcomm.so

TechnosoftMotor_SYS_LIBS += TML_lib
TechnosoftMotor_SYS_LIBS += tmlcomm
TechnosoftMotor_SYS_LIBS += z

endif

# -------------------------------------------------------
# Sources (common to both modes)
# -------------------------------------------------------
TechnosoftMotor_SRCS += drvTmlMotor.cpp

# Install public header so external IOCs can use the library
INC += drvTmlMotor.h

# -------------------------------------------------------
# Libraries this support lib depends on
# -------------------------------------------------------
TechnosoftMotor_LIBS += motor
TechnosoftMotor_LIBS += asyn
TechnosoftMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
