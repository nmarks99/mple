#pragma once
#include "mple_adapter.hpp"
#include "trajectory.hpp"
#include "mple_adapter_registry.hpp"
#include <mutex>
#include <vector>
#include <iostream>
#include <stdexcept>
extern "C" {
#include "lua.h"
}

/// \brief MpleAdapter whose methods are implemented in a user provided lua script
class LuaMpleAdapter : public MpleAdapter {
  public:
    LuaMpleAdapter(const char* script_path, const char* macros);
    unsigned int num_axes() const override;
    std::vector<double> get_axis_positions() const override;
    std::vector<double> get_tcp_pose() const override;
    MpleModelConfig config() override;
    void stop() const override;
    bool moving() const override;
    bool move(const Trajectory& trajectory) const override;
    std::optional<std::vector<double>> fk(const std::vector<double>& joint_angles) const override;
    std::optional<std::vector<double>> ik(const std::vector<double>& tcp_pose) const override;

  private:
    lua_State* lua_state_;
    mutable std::recursive_mutex mutex_;

    std::vector<double> lua_table_to_vector_double(lua_State* L) const {
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

    template <typename T>
    void lua_push_args (T val) const {
        if constexpr (std::is_arithmetic_v<T>) {
            lua_pushnumber(lua_state_, val);
        } else if constexpr (std::is_same_v<T, std::vector<double>>) {
            // create lua table and fill in with waypoint
            lua_newtable(lua_state_);
            for (size_t i = 0; i < val.size(); i++) {
                lua_pushnumber(lua_state_, val[i]);
                lua_rawseti(lua_state_, -2, i + 1);
            }
        } else if constexpr (std::is_same_v<T, Trajectory>) {
            lua_newtable(lua_state_);
            for (size_t i = 0; i < val.size(); ++i) {
                const auto& pt = val[i];
                lua_newtable(lua_state_);

                lua_pushnumber(lua_state_, pt.time);
                lua_setfield(lua_state_, -2, "time");

                auto pushVec = [this](const std::vector<double>& v) {
                    lua_newtable(lua_state_);
                    for (size_t j = 0; j < v.size(); ++j) {
                        lua_pushnumber(lua_state_, v[j]);
                        lua_rawseti(lua_state_, -2, j + 1);
                    }
                };

                pushVec(pt.position);
                lua_setfield(lua_state_, -2, "position");
                pushVec(pt.velocity);
                lua_setfield(lua_state_, -2, "velocity");
                pushVec(pt.acceleration);
                lua_setfield(lua_state_, -2, "acceleration");

                lua_rawseti(lua_state_, -2, i + 1);
            }
        }
    }

    // Used for functions which return void (nil in lua)
    struct LuaReturnVoid {};

    template <typename T, typename... Args>
    std::optional<T> call_lua_method(const char* fn_name, Args... args) const {
        // Return value
        T value{};

        // Check that the function exists
        int result = lua_getglobal(lua_state_, fn_name);
        if (result != LUA_TFUNCTION) {
            std::cout << "Can't find function " << fn_name << std::endl;
            lua_pop(lua_state_, 1);
            return std::nullopt;
        }

        // Number of function arguments in the parameter pack
        constexpr std::size_t num_args = sizeof...(args);

        if constexpr (std::is_same_v<T, LuaReturnVoid>) {
            (lua_push_args(args), ...);

            lua_pcall(lua_state_, num_args, 1, 0);
            int return_type = lua_type(lua_state_, -1);
            value = LuaReturnVoid{};
            if (return_type != LUA_TNIL) {
                std::cout << "Return value ignored. " << std::string(fn_name) + " should return nil but is type " << return_type << "\n";
            }
        }

        if constexpr (std::is_same_v<T, bool>) {
            (lua_push_args(args), ...);

            lua_pcall(lua_state_, num_args, 1, 0);
            int return_type = lua_type(lua_state_, -1);
            if (return_type == LUA_TBOOLEAN) {
                value = lua_toboolean(lua_state_, -1);
            } else {
                lua_pop(lua_state_, 1);
                std::cout << std::string(fn_name) + " must return a bool, but doesn't\n";
                return std::nullopt;
                // throw std::runtime_error("move must return a bool, but doesn't");
            }
        }

        else if constexpr (std::is_arithmetic_v<T>) {
            (lua_push_args(args), ...);

            // Call the function
            lua_pcall(lua_state_, num_args, 1, 0);
            int return_type = lua_type(lua_state_, -1);

            // Checks that the return type is compatable with T
            if (return_type == LUA_TNUMBER) {
                value = lua_tonumber(lua_state_, -1);
            } else {
                lua_pop(lua_state_, 1);
                std::cerr << std::string(fn_name) + " must return a number, but doesn't\n";
                return std::nullopt;
            }
        }

        else if constexpr (std::is_same_v<T, std::vector<double>>) {
            (lua_push_args(args), ...);

            // Call the function
            lua_pcall(lua_state_, num_args, 1, 0);
            int return_type = lua_type(lua_state_, -1);

            // Checks that the return type is compatable with T
            if (return_type == LUA_TTABLE) {
                // TODO: ensure this succeeds
                value = lua_table_to_vector_double(lua_state_);
            } else {
                lua_pop(lua_state_, 1);
                // nil means generally that IK failed, otherwise perhaps user's lua method is just
                // using the wrong return type, so we say so in the error message here
                if (return_type != LUA_TNIL) {
                    std::cerr << std::string(fn_name) + " must return a table of numbers, but doesn't\n";
                }
                return std::nullopt;
            }
        }

        else if constexpr (std::is_same_v<T, MpleModelConfig>) {
            if (num_args > 0) {
                return std::nullopt;
            }

            // Call the function
            lua_pcall(lua_state_, num_args, 1, 0);
            int return_type = lua_type(lua_state_, -1);

            // Checks that the return type is compatable with T
            if (return_type == LUA_TSTRING) {
                auto str = lua_tostring(lua_state_, -1);
                value = MpleModelConfig{str};
            } else {
                lua_pop(lua_state_, 1);
                std::cerr << std::string(fn_name) + " must return a string, but doesn't\n";
                return std::nullopt;
            }
        }

        lua_pop(lua_state_, 1);
        return value;
    }
};
