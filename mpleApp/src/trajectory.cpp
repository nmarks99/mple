#include "trajectory.hpp"

#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>

#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/parametrizer/const_accel.hpp>
#include <toppra/solver/seidel.hpp>

Trajectory build_trajectory(const std::vector<std::vector<double>>& states,
    double max_velocity, double max_acceleration) {

    if (states.size() < 2) {
        throw std::invalid_argument("At least two waypoints are required");
    }
    if (!std::isfinite(max_velocity) || max_velocity <= 0.0) {
        throw std::invalid_argument("Maximum velocity must be finite and positive");
    }
    if (!std::isfinite(max_acceleration) || max_acceleration <= 0.0) {
        throw std::invalid_argument("Maximum acceleration must be finite and positive");
    }

    const size_t state_size = states[0].size();

    // Store waypoints from user in toppra::Vectors
    toppra::Vectors waypoints;
    for (auto& wp : states) {
        waypoints.push_back(Eigen::Map<const toppra::Vector>(wp.data(), wp.size()));
    }

    // Assign parameter values s to the geometric waypoints
    // TODO: This would be better: s[i] = s[i - 1] + norm(q[i] - q[i - 1])
    toppra::Vector s_vec = toppra::Vector::LinSpaced(waypoints.size(), 0.0, waypoints.size() - 1);

    // Create the cubic spline describing joint positions along the geometric path
    toppra::BoundaryCond natural{2, toppra::Vector::Zero(state_size)};
    auto path = std::make_shared<toppra::PiecewisePolyPath>(
        toppra::PiecewisePolyPath::CubicSpline(
            waypoints, s_vec, toppra::BoundaryCondFull{natural, natural}));

    // Set contraints for the velocity and acceleration
    const toppra::Vector velocity_limit = toppra::Vector::Constant(state_size, max_velocity);
    const toppra::Vector acceleration_limit = toppra::Vector::Constant(state_size, max_acceleration);
    toppra::LinearConstraintPtrs constraints{
        std::make_shared<toppra::constraint::LinearJointVelocity>(
            -velocity_limit, velocity_limit),
        std::make_shared<toppra::constraint::LinearJointAcceleration>(
            -acceleration_limit, acceleration_limit),
    };

    // Compute a time-optimal path scaling s(t) subject to the joint limits
    toppra::algorithm::TOPPRA algorithm{constraints, path};
    algorithm.solver(std::make_shared<toppra::solver::Seidel>());
    const auto result = algorithm.computePathParametrization(0.0, 0.0);
    if (result != toppra::ReturnCode::OK) {
        std::stringstream msg;
        msg << "TOPP-RA could not parameterize the path: "
                  << algorithm.getErrorMessage() << '\n';
        throw std::runtime_error(msg.str());
    }

    // Construct the time-domain trajectory q(t) from the parameterization.
    const auto& data = algorithm.getParameterizationData();
    toppra::parametrizer::ConstAccel trajectory{path, data.gridpoints, data.parametrization};
    const auto& times = trajectory.getTimes();

    Trajectory traj{};
    for (Eigen::Index index = 0; index < times.size(); ++index) {
        TrajectoryPoint pt{};
        const double time = times[index];
        pt.time = time;
        const auto pos = trajectory.eval_single(time, 0);
        pt.position.assign(pos.begin(), pos.end());
        const auto vel = trajectory.eval_single(time, 1);
        pt.velocity.assign(vel.begin(), vel.end());
        const auto acc = trajectory.eval_single(time, 2);
        pt.acceleration.assign(acc.begin(), acc.end());
        traj.push_back(pt);
    }

    return traj;
}
