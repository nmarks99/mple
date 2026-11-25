# NOTE: paths relative to where they get installed (e.g. ./O.rhel9-x86_64)

TOP=../..

include $(TOP)/configure/CONFIG

# Requires C++17
USR_CXXFLAGS += -std=c++17
USR_CPPFLAGS += $(MPLE_SUPPORT_CPPFLAGS)

# Define IOC library name
LIBRARY_IOC += mple

# install mpleSupport.dbd into <top>/dbd
DBD += mpleSupport.dbd
mpleSupport_DBD += mple.dbd

# headers to install
INC += mple/adapter.hpp
INC += mple/adapter_registry.hpp
INC += mple/trajectory.hpp

USR_INCLUDES += $(MPLE_SUPPORT_INCLUDES)

# Libraries needed for mple
mple_LIBS += toppra
mple_LIBS += ompl
mple_SYS_LIBS += boost_serialization
mple_SYS_LIBS += boost_filesystem
mple_LIBS += asyn
mple_LIBS += lua
mple_LIBS += pvxs
mple_LIBS += pinocchio_default
mple_LIBS += pinocchio_parsers
mple_LIBS += pinocchio_collision
mple_LIBS += coal
mple_LIBS += $(EPICS_BASE_IOC_LIBS)

# source files to be compiled and added to the library
mple_SRCS += driver.cpp
mple_SRCS += rpc.cpp
mple_SRCS += trajectory.cpp
mple_SRCS += kinematics.cpp
mple_SRCS += collisions.cpp
mple_SRCS += planner.cpp
mple_SRCS += scene.cpp
mple_SRCS += lua_adapter.cpp
mple_SRCS += adapter_registry.cpp

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
