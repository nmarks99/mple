TOP=../../..

include $(TOP)/configure/CONFIG

USR_CXXFLAGS += -std=c++17
USR_INCLUDES += -I$(TOP)/deps/eigen
USR_INCLUDES += -I$(TOP)/deps/json/single_include
SRC_DIRS += $(TOP)/mpleApp/src

TESTPROD_HOST += testCollision
testCollision_SRCS += test_collisions.cpp
testCollision_SRCS += collisions.cpp

TESTPROD_HOST += testJson
testJson_SRCS += test_json.cpp
testJson_SRCS += collisions.cpp
testJson_LIBS += Com
TESTS += testJson

TESTPROD_HOST += testTrajectory
testTrajectory_SRCS += test_trajectory.cpp
testTrajectory_LIBS += Com
TESTS += testTrajectory
TESTSCRIPTS_HOST += $(TESTS:%=%.t)

include $(TOP)/configure/RULES
