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

// int LuaMpleAdapter::get_num_joints() const {
    // std::lock_guard<std::recursive_mutex> lock(mutex_);
//
    // int value = 0;
//
    // int result = lua_getglobal(lua_state_, "get_num_joints");
    // if (result == LUA_TFUNCTION) {
        // lua_pcall(lua_state_, 0, 1, 0);
        // int return_type = lua_type(lua_state_, -1);
        // if (return_type == LUA_TNUMBER) {
            // value = lua_tonumber(lua_state_, -1);
        // } else {
            // lua_pop(lua_state_, 1);
            // throw std::runtime_error("get_num_joints must return an integer, but doesn't");
        // }
    // } else {
        // lua_pop(lua_state_, 1);
        // throw std::runtime_error("get_num_joints implementation not found");
    // }
//
    // lua_pop(lua_state_, 1);
    // return value;
// }

std::vector<double> LuaMpleAdapter::get_joint_positions() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::vector<double> value = {};

    int result = lua_getglobal(lua_state_, "get_joint_positions");
    if (result == LUA_TFUNCTION) {
        lua_pcall(lua_state_, 0, 1, 0);
        int return_type = lua_type(lua_state_, -1);
        if (return_type == LUA_TTABLE) {
            value = extract_table_to_vector(lua_state_);
        } else {
            lua_pop(lua_state_, 1);
            throw std::runtime_error("get_joint_positions must return a table of numbers, but doesn't");
        }
    } else {
        lua_pop(lua_state_, 1);
        throw std::runtime_error("get_joint_positions implementation not found");
    }
    lua_pop(lua_state_, 1);
    return value;
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

    bool retval = false;

    for (auto& wp : waypoints) {
        int result = lua_getglobal(lua_state_, "move");
        if (result == LUA_TFUNCTION) {
            // create lua table and fill in with waypoint
            lua_newtable(lua_state_);
            for (size_t i = 0; i < wp.size(); i++) {
                lua_pushnumber(lua_state_, wp[i]);
                lua_rawseti(lua_state_, -2, i + 1);
            }

            // push speed and accelerations parameters
            lua_pushnumber(lua_state_, speed);
            lua_pushnumber(lua_state_, accel);

            // call move(wp, speed, accel)
            lua_pcall(lua_state_, 3, 1, 0);
            int return_type = lua_type(lua_state_, -1);
            if (return_type == LUA_TBOOLEAN) {
                retval = lua_toboolean(lua_state_, -1);
            } else {
                lua_pop(lua_state_, 1);
                throw std::runtime_error("move must return a bool, but doesn't");
            }
        } else {
            lua_pop(lua_state_, 1);
            throw std::runtime_error("move implementation not found");
        }

        lua_pop(lua_state_, 1);

        if (!retval) {
            break;
        }
    }

    return retval;
}

std::optional<std::vector<double>> LuaMpleAdapter::fk(const std::vector<double>& joint_angles) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::vector<double> value = {};

    int result = lua_getglobal(lua_state_, "fk");
    if (result == LUA_TFUNCTION) {

        // TODO: check that joint_angles.size() == get_num_joints()

        // create lua table and fill it with joint_angles vector
        lua_newtable(lua_state_);
        for (size_t i = 0; i < joint_angles.size(); i++) {
            lua_pushnumber(lua_state_, joint_angles[i]);
            lua_rawseti(lua_state_, -2, i + 1);
        }

        lua_pcall(lua_state_, 1, 1, 0);
        int return_type = lua_type(lua_state_, -1);
        if (return_type == LUA_TTABLE) {
            value = extract_table_to_vector(lua_state_);
            if (value.size() != 3) {
                lua_pop(lua_state_, 1);
                throw std::runtime_error("Returned array from fk must be length 3 (x,y,z) but is only length " + std::to_string(value.size()));
            }
        } else {
            lua_pop(lua_state_, 1);
            throw std::runtime_error("fk must return a table of numbers, but doesn't");
        }
    } else {
        lua_pop(lua_state_, 1);
        // Since the lua adapter doesn't provide it, call the base implementation of forward kinematics
        return MpleAdapter::fk(joint_angles);
    }
    lua_pop(lua_state_, 1);
    return value;
}

std::optional<std::vector<double>> LuaMpleAdapter::ik(const std::vector<double>& tcp_pose) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::vector<double> value = {};

    int result = lua_getglobal(lua_state_, "ik");
    if (result == LUA_TFUNCTION) {

        // TODO: check that tcp_pose.size() == 6

        // create lua table and fill it with joint_angles vector
        lua_newtable(lua_state_);
        for (size_t i = 0; i < tcp_pose.size(); i++) {
            lua_pushnumber(lua_state_, tcp_pose[i]);
            lua_rawseti(lua_state_, -2, i + 1);
        }

        lua_pcall(lua_state_, 1, 1, 0);
        int return_type = lua_type(lua_state_, -1);
        if (return_type == LUA_TTABLE) {
            value = extract_table_to_vector(lua_state_);
        } else {
            lua_pop(lua_state_, 1);
            throw std::runtime_error("ik must return a table of numbers, but doesn't");
        }
    } else {
        lua_pop(lua_state_, 1);
        // Since the lua adapter doesn't provide it, call the base implementation of inverse kinematics
        return MpleAdapter::ik(tcp_pose);
    }
    lua_pop(lua_state_, 1);
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

