#include <iocsh.h>
#include <epicsExport.h>
#include <luaEpics.h>
#include <macLib.h>
#include <stdexcept>
#include <iostream>
#include <cmath>

#include "lua_mple_adapter.hpp"

LuaMpleAdapter::LuaMpleAdapter(const char* script_path, const char* macros) {
    lua_state_ = luaCreateState();
    if (!lua_state_) {
        throw std::runtime_error("Error creating lua state");
    }

    // Push macros to globals in lua state
    luaLoadMacros(lua_state_, macros);

    int err = luaLoadScript(lua_state_, script_path);
    if (err) {
        const char* err = lua_tostring(lua_state_, -1);
        printf("\t%s\n", err);
        throw std::runtime_error("Error compiling script file: " + std::string(script_path));
    }

    // TODO:
    // auto config = call_lua_method<SystemModel>("config");
}

MpleModelConfig LuaMpleAdapter::config() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto ret = call_lua_method<MpleModelConfig>("config");
    if (!ret) {
        throw std::runtime_error("Error calling lua config method");
    }
    return ret.value();
}

std::vector<double> LuaMpleAdapter::get_state() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto value = call_lua_method<std::vector<double>>("get_state");
    return value.value_or(std::vector<double>{0.0});
}

// // TODO: do we need something like this?
// std::vector<double> LuaMpleAdapter::get_tcp_pose() const {
    // std::lock_guard<std::recursive_mutex> lock(mutex_);
    // return {...};
// }

void LuaMpleAdapter::stop() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    call_lua_method<LuaReturnVoid>("stop");
    return;
}

bool LuaMpleAdapter::moving() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto ret = call_lua_method<bool>("moving");
    if (!ret) {
        throw std::runtime_error("Error calling lua moving method");
    }
    return ret.value();
};

bool LuaMpleAdapter::move(const Trajectory& trajectory) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto ret = call_lua_method<bool>("move", trajectory);
    if (!ret) {
        throw std::runtime_error("Error calling lua move method");
    }
    return ret.value();
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
        // TODO: fix hardcoded 6
        if (value->size() != static_cast<size_t>(6))  {
            throw std::runtime_error("Returned array from ik must be equal to the number of joints");
        }
    } else {
        return MpleAdapter::ik(tcp_pose);
    }
    return value;
}

extern "C" void LuaMpleAdapterConfig(const char* name, const char* script_path, const char* macros) {
    register_adapter(new LuaMpleAdapter(script_path, macros), name);
}

static const iocshArg LuaMpleAdapterArg0 = {"name", iocshArgString};
static const iocshArg LuaMpleAdapterArg1 = {"script_path", iocshArgString};
static const iocshArg LuaMpleAdapterArg2 = {"macros", iocshArgString};
static const iocshArg* const LuaMpleAdapterArgs[3] = {&LuaMpleAdapterArg0, &LuaMpleAdapterArg1, &LuaMpleAdapterArg2};
static const iocshFuncDef LuaMpleAdapterFuncDef = {"LuaMpleAdapterConfig", 3, LuaMpleAdapterArgs};

static void LuaMpleAdapterCallFunc(const iocshArgBuf* args) { LuaMpleAdapterConfig(args[0].sval, args[1].sval, args[2].sval); }

void LuaMpleAdapterRegister(void) { iocshRegister(&LuaMpleAdapterFuncDef, LuaMpleAdapterCallFunc); }

extern "C" {
epicsExportRegistrar(LuaMpleAdapterRegister);
}

