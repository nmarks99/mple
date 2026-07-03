#include <vector>
#include "iocsh.h"
#include "epicsExport.h"
#include "robot_adapter.hpp"
#include "robot_adapter_registry.hpp"

void log(const char* msg) {
    printf("[ExampleRobotAdapter] %s\n", msg);
}

class ExampleRobotAdapter : public RobotAdapter {
  public:
    ExampleRobotAdapter() {};

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


extern "C" void ExampleRobotAdapterConfig(const char* name) {
    register_adapter(new ExampleRobotAdapter(), name);
}

static const iocshArg ExampleRobotAdapterArg0 = {"name", iocshArgString};
static const iocshArg* const ExampleRobotAdapterArgs[1] = {&ExampleRobotAdapterArg0};
static const iocshFuncDef ExampleRobotAdapterFuncDef = {"ExampleRobotAdapterConfig", 1, ExampleRobotAdapterArgs};

static void ExampleRobotAdapterCallFunc(const iocshArgBuf* args) { ExampleRobotAdapterConfig(args[0].sval); }

void ExampleRobotAdapterRegister(void) { iocshRegister(&ExampleRobotAdapterFuncDef, ExampleRobotAdapterCallFunc); }

extern "C" {
epicsExportRegistrar(ExampleRobotAdapterRegister);
}
