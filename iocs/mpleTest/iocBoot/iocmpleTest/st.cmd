# ../../bin/${EPICS_HOST_ARCH}/mpleTest st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocmpleTestLinux.dbd")
iocmpleTestLinux_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("IOCSH_PS1", "$(IOC)>")
epicsEnvSet("PREFIX", "mpleTest:")
epicsEnvSet("LUA_SCRIPT_PATH", ".")

# ExampleRobotAdapterConfig("adapter1")
LuaRobotAdapterConfig("adapter1", "adapter1.lua")
MpleDriverConfig("planner1", "adapter1")

# # Load simulated motor (Requires motor module)
# epicsEnvSet("INSTANCE", "motorSim")
# epicsEnvSet("NUM_AXES", "2")
# epicsEnvSet("DHLM", "100000000")
# epicsEnvSet("DLLM", "-100000000")
# epicsEnvSet("HIGH_LIM", "100000000")
# epicsEnvSet("LOW_LIM", "-100000000")
# motorSimCreate($(CONTROLLER=0), 0, $(LOW_LIM=-32000), $(HIGH_LIM=32000), $(HOME_POS=0), 1, $(NUM_AXES=1))
# drvAsynMotorConfigure("$(INSTANCE)$(CONTROLLER=0)", "$(INSTANCE)", $(CONTROLLER=0), $(NUM_AXES=1))
# dbLoadTemplate("motorSim.substitutions", "P=$(PREFIX), DTYP='asynMotor', PORT=$(INSTANCE)$(CONTROLLER=0),DHLM=$(DHLM=320.0),DLLM=$(DLLM=-320.0)")

luaLoadFile("rr_robot.lua", "P=$(PREFIX),STATE=LUASTATE0")

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
