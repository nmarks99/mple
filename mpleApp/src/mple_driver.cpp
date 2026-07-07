#include <iocsh.h>

#include "epicsExport.h"
#include "epicsThread.h"

#include "rpc_handler.hpp"
#include "mple_driver.hpp"
// #include "services.hpp"

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

    // Get the IOC prefix
    std::string prefix = std::getenv("PREFIX");
    if (!prefix.size()) {
        throw std::runtime_error("PREFIX environment variable not found, no custom PV loaded\n");
    }

    // Find MpleAdapter by name
    // This implements the robot specific functions needed by the path planner
    auto* adapter = find_adapter(adapter_name);
    if (!adapter) {
        throw std::runtime_error("Failed to find adapter: " + std::string(adapter_name));
    }

    // Create asyn parameters
    createParam("MT_ACTIVE", asynParamInt32, &idx_mt_active_);
    createParam("MT_ERROR", asynParamInt32, &idx_mt_error_);

    // RPCHandler sets up PVXS PVs with RPC callbacks
    rpc_handler_ = std::make_unique<RPCHandler>(this, adapter, prefix);

    // Create polling thread
    epicsThreadCreate("MpleDriverPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void MpleDriver::poll() {
    while (true) {
        lock();

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
