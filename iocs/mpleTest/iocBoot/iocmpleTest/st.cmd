# ../../bin/${EPICS_HOST_ARCH}/mpleTest st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocmpleTestLinux.dbd")
iocmpleTestLinux_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("IOCSH_PS1", "$(IOC)>")
epicsEnvSet("PREFIX", "mpleTest:")

# Simulated planar RR robot arm
luaRunFile("rr_robot.lua", "P=$(PREFIX),STATE=LUASTATE0")

# Load the adapter for our robot and create a mple instance
LuaMpleAdapterConfig("rr_robot", "rr_robot_adapter.lua")
MpleConfig("rr_robot", "$(PREFIX)mple:", "obstacles.json")

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
