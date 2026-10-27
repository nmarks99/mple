#pragma once
#include <vector>
#include <optional>

/// \brief Abstract base class for implementing a robot specific adapter for mple
class MpleAdapter {
  public:
    virtual ~MpleAdapter() = default;

    // TODO:
    // virtual ? get_joint_limits() const = 0;

    /// \brief Gets the number of axes in the system
    virtual unsigned int num_axes() const = 0;

    /// \brief Gets the current joint positions
    /// \return vector of joint positions
    virtual std::vector<double> get_axis_positions() const = 0;

    /// \brief Gets the current pose of the tool center point (TCP) in 3D space
    /// \return vector of the current pose (x,y,z,rx,ry,rz)
    virtual std::vector<double> get_tcp_pose() const = 0;

    /// \brief Stops all active motion of the system
    virtual void stop() const = 0;

    /// \brief Tells if the system is currenty in motion or not
    virtual bool moving() const = 0;

    /// \brief Starts execution of a time-scaled joint trajectory.
    virtual bool move(const std::vector<double>& trajectory) const = 0;

    /// \brief Computes forward kinematics for the system
    ///
    /// Computes the TCP pose resulting from the given joint angles.
    /// \todo TODO: This method must compute FK itself. Base classes can optionally override this,
    /// however if they do not and choose to call it here, this method must know how to compute
    /// FK from the system geometry, likely via URDF or DH parameters.
    virtual std::optional<std::vector<double>> fk(const std::vector<double>& joint_angles) const {
        if (joint_angles.size() != static_cast<size_t>(num_axes())) {
            return std::nullopt;
        }
        return std::nullopt;
    };

    /// \brief Computes inverse kinematics for the system
    ///
    /// Computes the joint angles required to achieve the requested TCP pose.
    /// \todo TODO: This method must compute IK itself. Base classes can optionally override this,
    /// however if they do not and choose to call it directly, this method must know how to compute
    /// IK from the system geometry, likely via URDF or DH parameters.
    virtual std::optional<std::vector<double>> ik(const std::vector<double>& tcp_pose) const {
        if (tcp_pose.size() != static_cast<size_t>(num_axes())) {
            return std::nullopt;
        }
        return std::nullopt;
    };
};
