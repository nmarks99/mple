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

USR_INCLUDES += $(MPLE_SUPPORT_INCLUDES)

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
TESTPROD_HOST += testFK
testFK_SRCS += test_fk.cpp
testFK_SRCS += kinematics.cpp
testFK_LIBS += Com
testFK_LIBS += pinocchio_parsers
testFK_LIBS += pinocchio_default
TESTS += testFK

TESTSCRIPTS_HOST += $(TESTS:%=%.t)

include $(TOP)/configure/RULES
