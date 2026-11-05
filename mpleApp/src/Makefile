# NOTE: paths relative to where they get installed (e.g. ./O.rhel9-x86_64)

TOP=../..

include $(TOP)/configure/CONFIG

# Requires C++17
USR_CXXFLAGS += -std=c++17
USR_CPPFLAGS += -DBOOST_MPL_LIMIT_LIST_SIZE=30
USR_CPPFLAGS += -DBOOST_MPL_LIMIT_VECTOR_SIZE=30
USR_CPPFLAGS += -DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
USR_CPPFLAGS += -DBOOST_FUSION_INVOKE_MAX_ARITY=12

# Define IOC library name
LIBRARY_IOC += mple

# install mpleSupport.dbd into <top>/dbd
DBD += mpleSupport.dbd
mpleSupport_DBD += mple.dbd

# headers to install
INC += mple_adapter.hpp
INC += mple_system_model.hpp
INC += trajectory.hpp
INC += kinematics.hpp
INC += mple_adapter_registry.hpp

USR_INCLUDES += $(MPLE_SUPPORT_INCLUDES)

# Libraries needed for mple
mple_LIBS += toppra
mple_LIBS += ompl
mple_SYS_LIBS += boost_serialization
mple_LIBS += asyn
mple_LIBS += lua
mple_LIBS += pvxs
mple_LIBS += pinocchio_default
mple_LIBS += pinocchio_parsers
mple_LIBS += $(EPICS_BASE_IOC_LIBS)

# source files to be compiled and added to the library
mple_SRCS += mple_driver.cpp
mple_SRCS += trajectory.cpp
mple_SRCS += kinematics.cpp
mple_SRCS += collisions.cpp
mple_SRCS += lua_mple_adapter.cpp
mple_SRCS += mple_adapter_registry.cpp

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
