# NOTE: paths relative to where they get installed (e.g. ./O.rhel9-x86_64)

TOP=../..

include $(TOP)/configure/CONFIG

# Requires C++17
USR_CXXFLAGS += -std=c++17

# Define IOC library name
LIBRARY_IOC += mple

# install mpleSupport.dbd into <top>/dbd
DBD += mpleSupport.dbd
mpleSupport_DBD += mple.dbd

# headers to install
INC += mple_adapter.hpp
INC += mple_adapter_registry.hpp

# Eigen
USR_INCLUDES += -I$(TOP)/deps/eigen
USR_INCLUDES += -I$(TOP)/deps/json/single_include

# TOPP-RA
TOPPRA = $(TOP)/deps/toppra/install/$(T_A)
USR_INCLUDES += -isystem $(TOPPRA)/include
toppra_DIR = $(TOPPRA)/lib
LIB_INSTALLS += $(TOPPRA)/lib/libtoppra.so

# OMPL
OMPL = $(TOP)/deps/ompl/install/$(T_A)
USR_INCLUDES += -isystem $(OMPL)/include/ompl-2.0
ompl_DIR = $(OMPL)/lib
LIB_INSTALLS += $(OMPL)/lib/libompl.so
LIB_INSTALLS += $(OMPL)/lib/libompl.so.19
LIB_INSTALLS += $(OMPL)/lib/libompl.so.2.0.2

# Libraries needed for mple
mple_LIBS += toppra
mple_LIBS += ompl
mple_LIBS += asyn
mple_LIBS += lua
mple_LIBS += pvxs
mple_LIBS += $(EPICS_BASE_IOC_LIBS)

# source files to be compiled and added to the library
mple_SRCS += mple_driver.cpp
mple_SRCS += trajectory.cpp
mple_SRCS += collisions.cpp
mple_SRCS += lua_mple_adapter.cpp
mple_SRCS += mple_adapter_registry.cpp

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
