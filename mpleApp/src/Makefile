# NOTE: paths relative to where they get installed (e.g. ./O.rhel9-x86_64)

TOP=../..

include $(TOP)/configure/CONFIG

# Define IOC library name
LIBRARY_IOC += mple

# install mpleSupport.dbd into <top>/dbd
DBD += mpleSupport.dbd
mpleSupport_DBD += mple.dbd

# headers to install
INC += mple_adapter.hpp
INC += mple_adapter_registry.hpp

# Libraries needed for mple
mple_LIBS += asyn
mple_LIBS += lua
mple_LIBS += pvxs
mple_LIBS += $(EPICS_BASE_IOC_LIBS)

# Eigen
USR_INCLUDES += -I$(TOP)/deps/eigen

# source files to be compiled and added to the library
mple_SRCS += mple_driver.cpp
mple_SRCS += rpc_handler.cpp
mple_SRCS += collisions.cpp
mple_SRCS += lua_mple_adapter.cpp
mple_SRCS += mple_adapter_registry.cpp

# Testing collisions
TESTPROD += testCollision
testCollision_SRCS += test_collisions.cpp
testCollision_SRCS += collisions.cpp

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
