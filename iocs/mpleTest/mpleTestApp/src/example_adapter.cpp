#include <vector>
#include "iocsh.h"
#include "epicsExport.h"
#include "mple_adapter.hpp"
#include "mple_adapter_registry.hpp"

void log(const char* msg) {
    printf("[ExampleMpleAdapter] %s\n", msg);
}

class ExampleMpleAdapter : public MpleAdapter {
  public:
    ExampleMpleAdapter() {};

    int get_num_joints() const override {
        log("Requesting number of joints");
        return 2;
    };

    std::vector<double> get_joint_positions() const override {
        log("Requesting joint positions");
        return {1.0, 2.0};
    }

    std::vector<double> get_tcp_pose() const override {
        log("Requesting TCP pose");
        return {0.3, 0.4};
    }

    void stop() const override {
        log("Stopping robot");
    }

    bool execute_trajectory(const std::vector<std::vector<double>>& wps, double speed, double accel) const override {
        log("Executing trajectory");
        return true;
    }
};


extern "C" void ExampleMpleAdapterConfig(const char* name) {
    register_adapter(new ExampleMpleAdapter(), name);
}

static const iocshArg ExampleMpleAdapterArg0 = {"name", iocshArgString};
static const iocshArg* const ExampleMpleAdapterArgs[1] = {&ExampleMpleAdapterArg0};
static const iocshFuncDef ExampleMpleAdapterFuncDef = {"ExampleMpleAdapterConfig", 1, ExampleMpleAdapterArgs};

static void ExampleMpleAdapterCallFunc(const iocshArgBuf* args) { ExampleMpleAdapterConfig(args[0].sval); }

void ExampleMpleAdapterRegister(void) { iocshRegister(&ExampleMpleAdapterFuncDef, ExampleMpleAdapterCallFunc); }

extern "C" {
epicsExportRegistrar(ExampleMpleAdapterRegister);
}
