#pragma once
#include <vector>
#include <optional>
#include <string>

#include "trajectory.hpp"

/// \brief Stores configuration information needed to build
/// the MPLE system model.
struct MpleModelConfig {
    std::string description_dir;
    std::string urdf_path;
    std::string srdf_path; // optional, empty disables self-collision checking
    std::string base_frame;
    std::string ee_frame;
};

/// \brief Abstract base class for implementing a system specific adapter for MPLE
class MpleAdapter {
  public:
    virtual ~MpleAdapter() = default;

    /// \brief Gets all the needed information about the system
    virtual MpleModelConfig config() = 0;

    /// \brief Gets the current system state (e.g. joint positions, etc.)
    virtual std::vector<double> get_state() const = 0;

    /// \brief Stops all active motion of the system
    virtual void stop() const = 0;

    /// \brief Tells if the system is currenty in motion or not
    virtual bool moving() const = 0;

    /// \brief True if the system executes trajectories with move, false if it uses move_step
    virtual bool supports_trajectory() const = 0;

    /// \brief Starts execution of a time-scaled joint trajectory.
    virtual bool move(const Trajectory& trajectory) const { return false; }

    /// \brief Starts synchronized motion to a single target configuration
    ///
    /// time is the segment duration, velocity is the peak speed of each joint
    /// and acceleration is the acceleration of each joint.
    virtual bool move_step(const TrajectoryPoint& target) const { return false; }

    /// \brief Computes forward kinematics for the system
    ///
    /// Computes the forward kinematics from the given state vector.
    /// If this method is not overriden but the derived class,
    /// higher layers (MpleDriver) must provide FK
    virtual std::optional<std::vector<double>> fk(const std::vector<double>& state) const {
        return std::nullopt;
    };

    /// \brief Computes inverse kinematics for the system
    ///
    /// Computes the inverse kinematics from the given pose.
    /// If this method is not overriden but the derived class,
    /// higher layers (MpleDriver) must provide IK
    virtual std::optional<std::vector<double>> ik(const std::vector<double>& pose) const {
        return std::nullopt;
    };
};
