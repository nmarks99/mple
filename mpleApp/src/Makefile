# NOTE: paths relative to where they get installed (e.g. ./O.rhel9-x86_64)

TOP=../..

include $(TOP)/configure/CONFIG

# Requires C++17
USR_CXXFLAGS += -std=c++17
USR_CPPFLAGS += -DBOOST_MPL_LIMIT_LIST_SIZE=30
USR_CPPFLAGS += -DBOOST_MPL_LIMIT_VECTOR_SIZE=30
USR_CPPFLAGS += -DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
USR_CPPFLAGS += -DBOOST_FUSION_INVOKE_MAX_ARITY=12
USR_CPPFLAGS += -DPINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
USR_CPPFLAGS += -DPINOCCHIO_WITH_URDFDOM
USR_CPPFLAGS += -DPINOCCHIO_URDFDOM_HEADERS_MAJOR_VERSION=3
USR_CPPFLAGS += -DPINOCCHIO_URDFDOM_HEADERS_MINOR_VERSION=0
USR_CPPFLAGS += -DPINOCCHIO_URDFDOM_HEADERS_PATCH_VERSION=1

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

# Pinocchio
PINOCCHIO = $(TOP)/deps/pinocchio/install/$(T_A)
USR_INCLUDES += -isystem $(PINOCCHIO)/include
pinocchio_default_DIR = $(PINOCCHIO)/lib
pinocchio_parsers_DIR = $(PINOCCHIO)/lib
LIB_INSTALLS += $(PINOCCHIO)/lib/libpinocchio_default.so
LIB_INSTALLS += $(PINOCCHIO)/lib/libpinocchio_default.so.4.1.0
LIB_INSTALLS += $(PINOCCHIO)/lib/libpinocchio_parsers.so
LIB_INSTALLS += $(PINOCCHIO)/lib/libpinocchio_parsers.so.4.1.0
LIB_INSTALLS += $(PINOCCHIO)/lib/liburdfdom_model.so
LIB_INSTALLS += $(PINOCCHIO)/lib/liburdfdom_model.so.6
LIB_INSTALLS += $(PINOCCHIO)/lib/liburdfdom_model.so.6.0.2
LIB_INSTALLS += $(PINOCCHIO)/lib/liburdfdom_sensor.so
LIB_INSTALLS += $(PINOCCHIO)/lib/liburdfdom_sensor.so.6
LIB_INSTALLS += $(PINOCCHIO)/lib/liburdfdom_sensor.so.6.0.2
LIB_INSTALLS += $(PINOCCHIO)/lib/liburdfdom_world.so
LIB_INSTALLS += $(PINOCCHIO)/lib/liburdfdom_world.so.6
LIB_INSTALLS += $(PINOCCHIO)/lib/liburdfdom_world.so.6.0.2
LIB_INSTALLS += $(PINOCCHIO)/lib/libconsole_bridge.so
LIB_INSTALLS += $(PINOCCHIO)/lib/libconsole_bridge.so.1.0
LIB_INSTALLS += $(PINOCCHIO)/lib/libtinyxml2.so
LIB_INSTALLS += $(PINOCCHIO)/lib/libtinyxml2.so.11
LIB_INSTALLS += $(PINOCCHIO)/lib/libtinyxml2.so.11.0.0

# Libraries needed for mple
mple_LIBS += toppra
mple_LIBS += ompl
mple_LIBS += pinocchio_parsers
mple_LIBS += pinocchio_default
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
