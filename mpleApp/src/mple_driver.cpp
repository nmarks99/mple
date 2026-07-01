#include <iocsh.h>

#include "epicsExport.h"
#include "epicsThread.h"

#include "mple_driver.hpp"
#include "rpc.hpp"

static void poll_thread_C(void* pPvt) {
    MpleDriver* pMpleDriver = (MpleDriver*)pPvt;
    pMpleDriver->poll();
}

constexpr int MAX_CONTROLLERS = 1;
constexpr int INTERFACE_MASK = asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask;
constexpr int INTERRUPT_MASK = asynInt32Mask | asynFloat64Mask | asynOctetMask;
constexpr int ASYN_FLAGS = ASYN_MULTIDEVICE | ASYN_CANBLOCK;

MpleDriver::MpleDriver(const char* port, const char* adapter_name)
    : asynPortDriver(port, MAX_CONTROLLERS, INTERFACE_MASK, INTERRUPT_MASK, ASYN_FLAGS, 1, 0, 0) {

    // lua_state_ = luaCreateState();
    // int status = luaLoadScript(lua_state_, adapter);
    // if (status) {
    // printf("Error compiling script file: %s\n", adapter);
    // const char* err = lua_tostring(lua_state_, -1);
    // printf("\t%s\n", err);
    // }

    // Get the IOC prefix
    std::string prefix = std::getenv("PREFIX");
    if (!prefix.size()) {
        throw std::runtime_error("PREFIX environment variable not found, no custom PV loaded\n");
    }

    // Find RobotAdapter by name
    // This implements the robot specific functions needed by the path planner
    auto* adapter = find_adapter(adapter_name);
    if (!adapter) {
        throw std::runtime_error("Failed to find robot adapter: " + std::string(adapter_name));
    }

    // Set up PVXS RPC callbacks for our custom PVs
    //
    // Validate
    validate_pv_.onRPC([this, adapter](pvxs::server::SharedPV& pv,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
        rpc::validate::handler(this, adapter, std::move(op), std::move(request_value));
    });
    pvxs::ioc::server().addPV(prefix + "Planner:Validate", validate_pv_);

    // Execute
    execute_pv_.onRPC([this, adapter](pvxs::server::SharedPV& pv,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
        rpc::execute::handler(this, adapter, std::move(op), std::move(request_value));
    });
    pvxs::ioc::server().addPV(prefix + "Planner:Execute", execute_pv_);

    // Create polling thread
    epicsThreadCreate("MpleDriverPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void MpleDriver::poll() {
    while (true) {
        lock();

        // if (lua_getglobal(lua_state_, "hello_world") == LUA_TFUNCTION) {
        // lua_pcall(lua_state_, 0, 0, 0);
        // } else {
        // printf("function hello_world does not exist in lua script\n");
        // }
        // static int count = 0;
        // printf("count = %d\n", count++);

        callParamCallbacks();
        unlock();
        epicsThreadSleep(1.0);
    }
}

// asynStatus MpleDriver::writeInt32(asynUser* pasynUser, epicsInt32 value) {
// int function = pasynUser->reason;
// callParamCallbacks();
// return comm_ok ? asynSuccess : asynError;
// }

// asynStatus MpleDriver::writeFloat64(asynUser* pasynUser, epicsFloat64 value) {
// // int function = pasynUser->reason;
// bool comm_ok = true;
// callParamCallbacks();
// return comm_ok ? asynSuccess : asynError;
// }

// register function for iocsh
extern "C" int MpleDriverConfig(const char* port, const char* adapter_name) {
    new MpleDriver(port, adapter_name);
    return asynSuccess;
}

static const iocshArg MpleDriverArg0 = {"Connection asyn port", iocshArgString};
static const iocshArg MpleDriverArg1 = {"Driver asyn port", iocshArgString};
static const iocshArg* const MpleDriverArgs[2] = {&MpleDriverArg0, &MpleDriverArg1};
static const iocshFuncDef MpleDriverFuncDef = {"MpleDriverConfig", 2, MpleDriverArgs};

static void MpleDriverCallFunc(const iocshArgBuf* args) { MpleDriverConfig(args[0].sval, args[1].sval); }

void MpleDriverRegister(void) { iocshRegister(&MpleDriverFuncDef, MpleDriverCallFunc); }

extern "C" {
epicsExportRegistrar(MpleDriverRegister);
}
