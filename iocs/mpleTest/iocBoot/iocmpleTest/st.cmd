# ../../bin/${EPICS_HOST_ARCH}/mpleTest st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocmpleTestLinux.dbd")
iocmpleTestLinux_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("IOCSH_PS1", "$(IOC)>")
epicsEnvSet("PREFIX", "mpleTest:")

# ExampleRobotAdapterConfig("adapter1")
LuaRobotAdapterConfig("adapter1", "adapter1.lua")
MpleDriverConfig("planner1", "adapter1")

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
