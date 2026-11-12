TOP=../

include $(TOP)/configure/CONFIG

USR_CXXFLAGS += -std=c++17
USR_CPPFLAGS += $(MPLE_SUPPORT_CPPFLAGS)

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
testCollisions_SRCS += scene.cpp
testCollisions_LIBS += pvxs
testCollisions_LIBS += Com
TESTS += testCollisions

TESTPROD_HOST += testCollisionsJson
testCollisionsJson_SRCS += test_collisions_json.cpp
testCollisionsJson_SRCS += scene.cpp
testCollisionsJson_LIBS += pvxs
testCollisionsJson_LIBS += Com
TESTS += testCollisionsJson

TESTPROD_HOST += testPinocchioCollisions
testPinocchioCollisions_SRCS += test_pinocchio_collisions.cpp
testPinocchioCollisions_LIBS += pinocchio_collision
testPinocchioCollisions_LIBS += coal
testPinocchioCollisions_LIBS += pinocchio_parsers
testPinocchioCollisions_LIBS += pinocchio_default
testPinocchioCollisions_SYS_LIBS += boost_filesystem
testPinocchioCollisions_SYS_LIBS += boost_serialization
testPinocchioCollisions_LIBS += Com
TESTS += testPinocchioCollisions

TESTPROD_HOST += testScenePva
testScenePva_SRCS += test_scene_pva.cpp
testScenePva_SRCS += scene.cpp
testScenePva_LIBS += pvxs
testScenePva_LIBS += Com
TESTS += testScenePva

USR_CPPFLAGS += -DMPLE_TEST_DESCRIPTION_DIR=\"$(abspath $(TOP)/description)\"
TESTPROD_HOST += testFK
testFK_SRCS += test_fk.cpp
testFK_SRCS += kinematics.cpp
testFK_LIBS += Com
testFK_LIBS += pinocchio_parsers
testFK_LIBS += pinocchio_default
TESTS += testFK

TESTPROD_HOST += testCollisionChecker
testCollisionChecker_SRCS += test_collision_checker.cpp
testCollisionChecker_SRCS += collisions.cpp
testCollisionChecker_SRCS += scene.cpp
testCollisionChecker_LIBS += pvxs
testCollisionChecker_LIBS += pinocchio_collision
testCollisionChecker_LIBS += coal
testCollisionChecker_LIBS += pinocchio_parsers
testCollisionChecker_LIBS += pinocchio_default
testCollisionChecker_LIBS += Com
testCollisionChecker_SYS_LIBS += boost_filesystem
testCollisionChecker_SYS_LIBS += boost_serialization
TESTS += testCollisionChecker

TESTSCRIPTS_HOST += $(TESTS:%=%.t)

include $(TOP)/configure/RULES
