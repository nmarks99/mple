TOP=../

include $(TOP)/configure/CONFIG

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

# ----------
USR_INCLUDES += -I$(TOP)/deps/eigen
USR_INCLUDES += -I$(TOP)/deps/json/include

TOPPRA = $(TOP)/deps/toppra/install/$(T_A)
USR_INCLUDES += -isystem $(TOPPRA)/include
toppra_DIR = $(TOPPRA)/lib

PINOCCHIO = $(TOP)/deps/pinocchio/install/$(T_A)
USR_INCLUDES += -isystem $(PINOCCHIO)/include
pinocchio_default_DIR = $(PINOCCHIO)/lib
pinocchio_parsers_DIR = $(PINOCCHIO)/lib
# ----------

SRC_DIRS += $(TOP)/mpleApp/src

TESTPROD_HOST += testTrajectory
testTrajectory_SRCS += test_trajectory.cpp
testTrajectory_SRCS += trajectory.cpp
testTrajectory_LIBS += Com
testTrajectory_LIBS += toppra
TESTS += testTrajectory

TESTPROD_HOST += testCollisions
testCollisions_SRCS += test_collisions.cpp
testCollisions_SRCS += collisions.cpp
testCollisions_LIBS += Com
TESTS += testCollisions

TESTPROD_HOST += testCollisionsJson
testCollisionsJson_SRCS += test_collisions_json.cpp
testCollisionsJson_SRCS += collisions.cpp
testCollisionsJson_LIBS += Com
TESTS += testCollisionsJson

USR_CPPFLAGS += -DMPLE_TEST_URDF_PATH=\"$(abspath $(TOP)/description/ur5e.urdf)\"
TESTPROD_HOST += testPinocchio
testPinocchio_SRCS += test_pinocchio.cpp
testPinocchio_LIBS += Com
testPinocchio_LIBS += pinocchio_parsers
testPinocchio_LIBS += pinocchio_default
TESTS += testPinocchio

TESTSCRIPTS_HOST += $(TESTS:%=%.t)

include $(TOP)/configure/RULES
