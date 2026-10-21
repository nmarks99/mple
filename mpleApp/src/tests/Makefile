TOP=../../..

include $(TOP)/configure/CONFIG

USR_CXXFLAGS += -std=c++17
USR_INCLUDES += -I$(TOP)/deps/eigen
USR_INCLUDES += -I$(TOP)/deps/json/single_include
SRC_DIRS += $(TOP)/mpleApp/src

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

TESTPROD_HOST += testTrajectory
testTrajectory_SRCS += test_trajectory.cpp
testTrajectory_LIBS += Com
TESTS += testTrajectory

TESTSCRIPTS_HOST += $(TESTS:%=%.t)

include $(TOP)/configure/RULES
