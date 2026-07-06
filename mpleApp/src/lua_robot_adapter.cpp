#include "lua_robot_adapter.hpp"
#include "iocsh.h"
#include "epicsExport.h"
#include "luaEpics.h"
#include <stdexcept>
#include <cmath>

std::vector<double> extract_table_to_vector(lua_State* L) {
    if (!lua_istable(L, -1)) {
        throw std::runtime_error("Top of stack is not a table.");
    }

    size_t array_len = lua_rawlen(L, -1);
    std::vector<double> result;
    result.reserve(array_len);

    for (size_t i = 1; i <= array_len; ++i) {
        // Pushes table[i] to index -1; table moves to index -2
        lua_rawgeti(L, -1, i);

        if (lua_isnumber(L, -1)) {
            result.push_back(lua_tonumber(L, -1));
        } else {
            lua_pop(L, 1); // Clean up the invalid element
            throw std::runtime_error("Non-numeric value at index " + std::to_string(i));
        }

        lua_pop(L, 1); // Pop the element
    }

    return result;
}

LuaRobotAdapter::LuaRobotAdapter(const char* script_path) {
    lua_state_ = luaCreateState();
    if (!lua_state_) {
        throw std::runtime_error("Error creating lua state");
    }

    int err = luaLoadScript(lua_state_, script_path);
    if (err) {
        throw std::runtime_error("Error compiling script file: " + std::string(script_path));
        const char* err = lua_tostring(lua_state_, -1);
        printf("\t%s\n", err);
        return;
    }
}

int LuaRobotAdapter::get_num_joints() const {
    std::lock_guard<std::mutex> lock(mutex_);

    int value = 0;

    int result = lua_getglobal(lua_state_, "get_num_joints");
    if (result == LUA_TFUNCTION) {
        lua_pcall(lua_state_, 0, 1, 0);
        int return_type = lua_type(lua_state_, -1);
        if (return_type == LUA_TNUMBER) {
            value = lua_tonumber(lua_state_, -1);
        } else {
            lua_pop(lua_state_, 1);
            throw std::runtime_error("get_num_joints must return an integer, but doesn't");
        }
    } else {
        lua_pop(lua_state_, 1);
        throw std::runtime_error("get_num_joints implementation not found");
    }

    lua_pop(lua_state_, 1);
    return value;
}

std::vector<double> LuaRobotAdapter::get_joint_positions() const {
    std::lock_guard<std::mutex> lock(mutex_);
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
        throw std::runtime_error("get_num_joints implementation not found");
    }
    lua_pop(lua_state_, 1);
    return value;
}

std::vector<double> LuaRobotAdapter::get_tcp_pose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {0.0, 0.0};
}

void LuaRobotAdapter::stop() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return;
}

bool LuaRobotAdapter::execute_trajectory(const std::vector<std::vector<double>>& waypoints, double speed, double accel) const {
    std::lock_guard<std::mutex> lock(mutex_);

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


extern "C" void LuaRobotAdapterConfig(const char* name, const char* script_path) {
    register_adapter(new LuaRobotAdapter(script_path), name);
}

static const iocshArg LuaRobotAdapterArg0 = {"name", iocshArgString};
static const iocshArg LuaRobotAdapterArg1 = {"script_path", iocshArgString};
static const iocshArg* const LuaRobotAdapterArgs[2] = {&LuaRobotAdapterArg0, &LuaRobotAdapterArg1};
static const iocshFuncDef LuaRobotAdapterFuncDef = {"LuaRobotAdapterConfig", 2, LuaRobotAdapterArgs};

static void LuaRobotAdapterCallFunc(const iocshArgBuf* args) { LuaRobotAdapterConfig(args[0].sval, args[1].sval); }

void LuaRobotAdapterRegister(void) { iocshRegister(&LuaRobotAdapterFuncDef, LuaRobotAdapterCallFunc); }

extern "C" {
epicsExportRegistrar(LuaRobotAdapterRegister);
}

