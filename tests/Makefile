TOP=../

include $(TOP)/configure/CONFIG

USR_CXXFLAGS += -std=c++17

TOPPRA = $(TOP)/deps/toppra/install/$(T_A)
USR_INCLUDES += -I$(TOP)/deps/eigen
USR_INCLUDES += -isystem $(TOPPRA)/include
USR_INCLUDES += -I$(TOP)/deps/json/include
toppra_DIR = $(TOPPRA)/lib

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

TESTSCRIPTS_HOST += $(TESTS:%=%.t)

include $(TOP)/configure/RULES
