# ../../bin/${EPICS_HOST_ARCH}/ppTest st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocppTestLinux.dbd")
iocppTestLinux_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("IOCSH_PS1", "$(IOC)>")
epicsEnvSet("PREFIX", "ppTest:")


ExampleRobotAdapterConfig("adapter1")

PathPlannerConfig("asdf", "test.lua")

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
