#include "lua_mple_adapter.hpp"
#include "iocsh.h"
#include "epicsExport.h"
#include "luaEpics.h"
#include <stdexcept>
#include <cmath>

LuaMpleAdapter::LuaMpleAdapter(const char* script_path) {
    lua_state_ = luaCreateState();
    if (!lua_state_) {
        throw std::runtime_error("Error creating lua state");
    }

    int err = luaLoadScript(lua_state_, script_path);
    if (err) {
        const char* err = lua_tostring(lua_state_, -1);
        printf("\t%s\n", err);
        throw std::runtime_error("Error compiling script file: " + std::string(script_path));
    }
}

int LuaMpleAdapter::get_num_joints() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto value = call_lua_method<int>("get_num_joints");
    return value.value_or(0);
}

std::vector<double> LuaMpleAdapter::get_joint_positions() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto value = call_lua_method<std::vector<double>>("get_joint_positions");
    return value.value_or(std::vector<double>{0.0});
}

std::vector<double> LuaMpleAdapter::get_tcp_pose() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return {0.0, 0.0};
}

void LuaMpleAdapter::stop() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return;
}


bool LuaMpleAdapter::execute_trajectory(const std::vector<std::vector<double>>& waypoints, double speed, double accel) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    bool ok = true;
    for (auto& wp : waypoints) {
        auto ret = call_lua_method<bool>("move", wp, speed, accel);
        if (!ret) {
            throw std::runtime_error("Error calling lua move method");
        }
        ok = ret.value();
        if (!ok) {
            break;
        }
    }
    return ok;
}

std::optional<std::vector<double>> LuaMpleAdapter::fk(const std::vector<double>& joint_angles) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    auto value = call_lua_method<std::vector<double>>("fk", joint_angles);
    if (value) {
        if (value->size() != 3)  {
            throw std::runtime_error(
                "Returned array from fk must be length 3 (x,y,z) but is only length " + std::to_string(value->size()));
        }
    } else {
        return MpleAdapter::fk(joint_angles);
    }
    return value;
}

std::optional<std::vector<double>> LuaMpleAdapter::ik(const std::vector<double>& tcp_pose) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    auto value = call_lua_method<std::vector<double>>("ik", tcp_pose);
    if (value) {
        if (value->size() != get_num_joints())  {
            throw std::runtime_error("Returned array from ik must be equal to the number of joints");
        }
    } else {
        return MpleAdapter::ik(tcp_pose);
    }
    return value;
}

extern "C" void LuaMpleAdapterConfig(const char* name, const char* script_path) {
    register_adapter(new LuaMpleAdapter(script_path), name);
}

static const iocshArg LuaMpleAdapterArg0 = {"name", iocshArgString};
static const iocshArg LuaMpleAdapterArg1 = {"script_path", iocshArgString};
static const iocshArg* const LuaMpleAdapterArgs[2] = {&LuaMpleAdapterArg0, &LuaMpleAdapterArg1};
static const iocshFuncDef LuaMpleAdapterFuncDef = {"LuaMpleAdapterConfig", 2, LuaMpleAdapterArgs};

static void LuaMpleAdapterCallFunc(const iocshArgBuf* args) { LuaMpleAdapterConfig(args[0].sval, args[1].sval); }

void LuaMpleAdapterRegister(void) { iocshRegister(&LuaMpleAdapterFuncDef, LuaMpleAdapterCallFunc); }

extern "C" {
epicsExportRegistrar(LuaMpleAdapterRegister);
}

