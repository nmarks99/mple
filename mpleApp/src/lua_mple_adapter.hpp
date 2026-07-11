#pragma once
#include "mple_adapter.hpp"
#include "mple_adapter_registry.hpp"
#include <mutex>
#include <vector>
extern "C" {
#include "lua.h"
}

class LuaMpleAdapter : public MpleAdapter {
  public:
    LuaMpleAdapter(const char* script_path);

    int get_num_joints() const override;

    std::vector<double> get_joint_positions() const override;

    std::vector<double> get_tcp_pose() const override;

    void stop() const override;

    bool execute_trajectory(const std::vector<std::vector<double>>& waypoints, double speed, double accel) const override;

    std::optional<std::vector<double>> fk(const std::vector<double>& joint_angles) const override;

    std::optional<std::vector<double>> ik(const std::vector<double>& tcp_pose) const override;

  private:
    lua_State* lua_state_;
    mutable std::mutex mutex_;
};
