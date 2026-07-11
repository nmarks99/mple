#pragma once
#include <vector>
#include <optional>
#include <iostream>

class MpleAdapter {
  public:
    virtual ~MpleAdapter() = default;
    virtual int get_num_joints() const = 0;
    // virtual ? get_joint_limits() const = 0;
    virtual std::vector<double> get_joint_positions() const = 0;
    virtual std::vector<double> get_tcp_pose() const = 0;
    virtual void stop() const = 0;
    // TODO:
    // virtual bool is_moving() const = 0;
    virtual bool execute_trajectory(const std::vector<std::vector<double>>& waypoints, double speed, double accel) const = 0;

    // TODO: if derived classes do not override fk and ik,
    // we should implement it here from provided DH parameters
    // or URDF
    virtual std::optional<std::vector<double>> fk(const std::vector<double>& joint_angles) const {
        // TODO: compute FK, return TCP pose
        // TODO: double lock!
        if (joint_angles.size() != static_cast<size_t>(get_num_joints())) {
            return std::nullopt;
        }
        return std::nullopt;
    };
    virtual std::optional<std::vector<double>> ik(const std::vector<double>& tcp_pose) const {
        // TODO: compute IK, return joint angles
        // TODO: double lock!
        if (tcp_pose.size() != static_cast<size_t>(6)) {
            return std::nullopt;
        }
        return std::nullopt;
    };
};
