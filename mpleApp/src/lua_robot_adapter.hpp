#pragma once
#include <vector>
#include "robot_adapter.hpp"
#include "robot_adapter.hpp"
#include "robot_adapter_registry.hpp"
extern "C" {
#include "lua.h"
}

class LuaRobotAdapter : public RobotAdapter {
  public:
    LuaRobotAdapter(const char* script_path);

    int get_num_joints() const override;

    std::vector<double> get_joint_positions() const override;

    std::vector<double> get_tcp_pose() const override;

    void stop() const override;

    bool execute_trajectory() const override;

private:
    lua_State* lua_state_;
};
