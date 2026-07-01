TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# PROD_NAME is used by ADCORE to add libraries
# DBD_PREFIX is used by ADCORE to add dbd files
PROD_NAME = mpleTest
DBD_PREFIX = iocmpleTest
PROD_IOC = $(PROD_NAME)

# Note that the order in which .dbd files and libraries are added to a list
# variable is important.  For .dbd files, things must be defined before they
# are referenced by a later .dbd file.  For libraries, things must be referenced
# before they are searched for; otherwise they won't be included.

##################################################################
# First, define $(DBD_NAME)_DBD and $(PROD_NAME)_LIBS from the modules
# defined in RELEASE
##################################################################
DBD_SUFFIX_Linux = Linux
DBD_SUFFIX_WIN32 = Win$(if $(findstring 32, $(T_A)),32,64)
DBD_NAME = $(DBD_PREFIX)$(DBD_SUFFIX_$(OS_CLASS))

DBD  += $(DBD_NAME).dbd

$(DBD_NAME)_DBD += base.dbd

ifdef ASYN
  $(DBD_NAME)_DBD += asyn.dbd
  $(DBD_NAME)_DBD += drvAsynSerialPort.dbd drvAsynIPPort.dbd drvVxi11.dbd devGpib.dbd
  $(PROD_NAME)_LIBS := asyn $($(PROD_NAME)_LIBS)
endif

ifdef LUA
  $(DBD_NAME)_DBD += luaSupport.dbd
  $(PROD_NAME)_LIBS := lua $($(PROD_NAME)_LIBS)
endif

ifdef PVXS
  $(DBD_NAME)_DBD += pvxsIoc.dbd
  $(PROD_NAME)_LIBS := pvxsIoc pvxs $($(PROD_NAME)_LIBS)
endif

ifdef MPLE
  $(DBD_NAME)_DBD += mpleSupport.dbd
  $(PROD_NAME)_LIBS := mple $($(PROD_NAME)_LIBS)
endif

$(PROD_NAME)_LIBS += $(EPICS_BASE_IOC_LIBS)

# Robot specific adapter for path planner
$(PROD_NAME)_SRCS  += example_adapter.cpp
$(DBD_NAME)_DBD += example_adapter.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
$(PROD_NAME)_SRCS  += $(DBD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
