# ../../bin/${EPICS_HOST_ARCH}/mpleTest st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocmpleTestLinux.dbd")
iocmpleTestLinux_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("IOCSH_PS1", "$(IOC)>")
epicsEnvSet("PREFIX", "mpleTest:")

# Simulated planar RR robot arm
luaLoadFile("rr_robot.lua", "P=$(PREFIX),STATE=LUASTATE0")

# Load the adapter for our robot and create a mple instance
LuaRobotAdapterConfig("rr_robot", "rr_robot_adapter.lua")
MpleDriverConfig("planner1", "rr_robot")
dbLoadRecords("$(MPLE)/db/mple.db", "P=$(PREFIX),PORT=planner1")

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
