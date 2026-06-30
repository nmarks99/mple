#include <iocsh.h>

#include "epicsExport.h"
#include "epicsThread.h"

#include "path_planner.hpp"

static void poll_thread_C(void* pPvt) {
    PathPlanner* pPathPlanner = (PathPlanner*)pPvt;
    pPathPlanner->poll();
}

constexpr int MAX_CONTROLLERS = 1;
constexpr int INTERFACE_MASK = asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask;
constexpr int INTERRUPT_MASK = asynInt32Mask | asynFloat64Mask | asynOctetMask;
constexpr int ASYN_FLAGS = ASYN_MULTIDEVICE | ASYN_CANBLOCK;

PathPlanner::PathPlanner(const char* port, const char* script_path)
    : asynPortDriver(port, MAX_CONTROLLERS, INTERFACE_MASK, INTERRUPT_MASK, ASYN_FLAGS, 1, 0, 0) {

    // lua_state_ = luaCreateState();
    // int status = luaLoadScript(lua_state_, script_path);
    // if (status) {
    // printf("Error compiling script file: %s\n", script_path);
    // const char* err = lua_tostring(lua_state_, -1);
    // printf("\t%s\n", err);
    // }

    std::string prefix = std::getenv("PREFIX");
    if (!prefix.size()) {
        throw std::runtime_error("PREFIX environment variable not found, no custom PV loaded\n");
    }

    validate_pv_.onRPC([](pvxs::server::SharedPV& pv,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& args) {
            // Response
            using namespace pvxs::members;
            pvxs::Value response = pvxs::TypeDef(pvxs::TypeCode::Struct, {
                Bool("valid"),
                String("description"),
            }).create();

            // args holds the request
            if (args["resolution"].valid()) {
                response["valid"] = true;
                response["description"] = "Success";
                if (auto res = args["resolution"].as<double>()) {
                    printf("Got Resolution = %lf\n", res);
                }
            } else {
                response["valid"] = false;
                response["description"] = "Something went wrong";
            }
            op->reply(response);
        }
    );
    pvxs::ioc::server().addPV(prefix + "Path:Validate", validate_pv_);

    epicsThreadCreate("PathPlannerPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void PathPlanner::poll() {
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

// asynStatus PathPlanner::writeInt32(asynUser* pasynUser, epicsInt32 value) {
// int function = pasynUser->reason;
// callParamCallbacks();
// return comm_ok ? asynSuccess : asynError;
// }

// asynStatus PathPlanner::writeFloat64(asynUser* pasynUser, epicsFloat64 value) {
// // int function = pasynUser->reason;
// bool comm_ok = true;
// callParamCallbacks();
// return comm_ok ? asynSuccess : asynError;
// }

// register function for iocsh
extern "C" int PathPlannerConfig(const char* port, const char* script) {
    PathPlanner* pPathPlanner = new PathPlanner(port, script);
    pPathPlanner = NULL;
    (void)pPathPlanner;
    return (asynSuccess);
}

static const iocshArg PathPlannerArg0 = {"Connection asyn port", iocshArgString};
static const iocshArg PathPlannerArg1 = {"Driver asyn port", iocshArgString};
static const iocshArg* const PathPlannerArgs[2] = {&PathPlannerArg0, &PathPlannerArg1};
static const iocshFuncDef PathPlannerFuncDef = {"PathPlannerConfig", 2, PathPlannerArgs};

static void PathPlannerCallFunc(const iocshArgBuf* args) { PathPlannerConfig(args[0].sval, args[1].sval); }

void PathPlannerRegister(void) { iocshRegister(&PathPlannerFuncDef, PathPlannerCallFunc); }

extern "C" {
epicsExportRegistrar(PathPlannerRegister);
}
