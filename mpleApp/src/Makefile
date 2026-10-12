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

# ompl
USR_INCLUDES += -I$(OMPL)/include/ompl-2.0
ompl_DIR = $(OMPL)/lib64

# Libraries needed for mple
mple_LIBS += ompl
mple_LIBS += asyn
mple_LIBS += lua
mple_LIBS += pvxs
mple_LIBS += $(EPICS_BASE_IOC_LIBS)

# source files to be compiled and added to the library
mple_SRCS += mple_driver.cpp
mple_SRCS += collisions.cpp
mple_SRCS += lua_mple_adapter.cpp
mple_SRCS += mple_adapter_registry.cpp

# Testing collisions
TESTPROD += testCollision
testCollision_SRCS += test_collisions.cpp
testCollision_SRCS += collisions.cpp

# Testing json
TESTPROD += testJson
testJson_SRCS += test_json.cpp
testJson_SRCS += collisions.cpp

TESTPROD += testTrajectory
testTrajectory_SRCS += test_trajectory.cpp
testTrajectory_LIBS += Com
TESTS += testTrajectory

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
