# ../../bin/${EPICS_HOST_ARCH}/mpleUR st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocmpleURLinux.dbd")
iocmpleURLinux_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("IOCSH_PS1", "$(IOC)>")
epicsEnvSet("PREFIX", "mpleUR:")

# UR robot support
iocshLoad("urRobot.iocsh", "IP=164.54.104.7")

# Load the adapter for our robot and create a mple instance
LuaMpleAdapterConfig("UR", "ur_adapter.lua", "P=$(PREFIX),MPLE=$(MPLE)")
MpleConfig("UR", "$(PREFIX)mple:", "obstacles.json")

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
