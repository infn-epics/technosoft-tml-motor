TOP = ../..
include $(TOP)/configure/CONFIG

# -------------------------------------------------------
# Support library: libTechnosoftMotor.so
# -------------------------------------------------------
LIBRARY_IOC = TechnosoftMotor

DBD += devTechnosoft.dbd

# -------------------------------------------------------
# Sources
# -------------------------------------------------------
USR_CPPFLAGS += -DUSE_TML_NATIVE
TechnosoftMotor_SRCS += tmlSerial.cpp
TechnosoftMotor_SRCS += drvTmlMotor.cpp
TechnosoftMotor_SYS_LIBS += z

# Install public header so external IOCs can use the library
INC += drvTmlMotor.h

# -------------------------------------------------------
# Libraries this support lib depends on
# -------------------------------------------------------
TechnosoftMotor_LIBS += motor
TechnosoftMotor_LIBS += asyn
TechnosoftMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
