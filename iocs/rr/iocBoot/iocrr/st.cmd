# ../../bin/${EPICS_HOST_ARCH}/rr st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocrrLinux.dbd")
iocrrLinux_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("IOCSH_PS1", "$(IOC)>")
epicsEnvSet("PREFIX", "rr:")

# Simulated planar RR robot arm
luaRunFile("rr_robot.lua", "P=$(PREFIX),STATE=LUASTATE0")

# Load the adapter for our robot and create a mple instance
LuaMpleAdapterConfig("rr_robot", "rr_robot_adapter.lua", "P=$(PREFIX),MPLE=$(MPLE)")
MpleConfig("rr_robot", "$(PREFIX)mple:", "scene.json")

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
