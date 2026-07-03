#pragma once
#include <vector>

class RobotAdapter {
  public:
    virtual ~RobotAdapter() = default;
    virtual int get_num_joints() const = 0;
    // virtual ? get_joint_limits() const = 0;
    virtual std::vector<double> get_joint_positions() const = 0;
    virtual std::vector<double> get_tcp_pose() const = 0;
    virtual void stop() const = 0;
    // TODO:
    // virtual bool is_moving() const = 0;
    virtual bool execute_trajectory(const std::vector<std::vector<double>>& waypoints, double speed, double accel) const = 0;
};
