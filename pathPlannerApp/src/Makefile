# NOTE: paths relative to where they get installed (e.g. ./O.rhel9-x86_64)

TOP=../..

include $(TOP)/configure/CONFIG

# Define IOC library name
LIBRARY_IOC += pathPlanner

# install pathPlannerSupport.dbd into <top>/dbd
DBD += pathPlannerSupport.dbd
pathPlannerSupport_DBD += path_planner.dbd

# source files to be compiled and added to the library
pathPlanner_SRCS += path_planner.cpp

# Libraries needed for pathPlanner
pathPlanner_LIBS += asyn
pathPlanner_LIBS += lua
pathPlanner_LIBS += pvxs
pathPlanner_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
