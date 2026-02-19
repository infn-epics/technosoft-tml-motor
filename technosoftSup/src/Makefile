TOP = ../..
include $(TOP)/configure/CONFIG

# -------------------------------------------------------
# Support library: libTechnosoftMotor.so
# -------------------------------------------------------
LIBRARY_IOC = TechnosoftMotor

DBD += devTechnosoft.dbd

# -------------------------------------------------------
# TML library paths
# -------------------------------------------------------
USR_CPPFLAGS += -I$(TML_LIB)/include
USR_LDFLAGS  += -L$(TML_LIB)/lib

# Install vendor shared libraries so downstream IOCs find them at runtime
LIB_INSTALLS += $(TML_LIB)/lib/libTML_lib.so
LIB_INSTALLS += $(TML_LIB)/lib/libtmlcomm.so

# -------------------------------------------------------
# Sources
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

TechnosoftMotor_SYS_LIBS += TML_lib
TechnosoftMotor_SYS_LIBS += tmlcomm
TechnosoftMotor_SYS_LIBS += z

include $(TOP)/configure/RULES
