# ../../bin/${EPICS_HOST_ARCH}/ppTest st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocppTestLinux.dbd")
iocppTestLinux_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("IOCSH_PS1", "$(IOC)>")
epicsEnvSet("PREFIX", "ppTest:")

# ExampleRobotAdapterConfig("adapter1")
LuaRobotAdapterConfig("adapter1", "adapter1.lua")
PathPlannerConfig("planner1", "adapter1")

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
