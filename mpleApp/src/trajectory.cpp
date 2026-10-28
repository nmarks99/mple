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

std::vector<double> build_trajectory(
    const std::vector<std::vector<double>>& joint_states, unsigned int num_axes,
    double max_velocity, double max_acceleration) {

    if (num_axes == 0) {
        throw std::invalid_argument("Number of axes must be greater than zero");
    }
    if (joint_states.size() < 2) {
        throw std::invalid_argument("At least two waypoints are required");
    }
    if (!std::isfinite(max_velocity) || max_velocity <= 0.0) {
        throw std::invalid_argument("Maximum velocity must be finite and positive");
    }
    if (!std::isfinite(max_acceleration) || max_acceleration <= 0.0) {
        throw std::invalid_argument("Maximum acceleration must be finite and positive");
    }

    // Store waypoints from user in toppra::Vectors
    toppra::Vectors waypoints;
    for (auto& wp : joint_states) {
        waypoints.push_back(Eigen::Map<const toppra::Vector>(wp.data(), wp.size()));
    }

    // Assign parameter values s to the geometric waypoints
    // TODO: This would be better: s[i] = s[i - 1] + norm(q[i] - q[i - 1])
    toppra::Vector s_vec = toppra::Vector::LinSpaced(waypoints.size(), 0.0, waypoints.size() - 1);

    // Create the cubic spline describing joint positions along the geometric path
    toppra::BoundaryCond natural{2, toppra::Vector::Zero(num_axes)};
    auto path = std::make_shared<toppra::PiecewisePolyPath>(
        toppra::PiecewisePolyPath::CubicSpline(
            waypoints, s_vec, toppra::BoundaryCondFull{natural, natural}));

    // Set contraints for the velocity and acceleration
    const toppra::Vector velocity_limit = toppra::Vector::Constant(num_axes, max_velocity);
    const toppra::Vector acceleration_limit = toppra::Vector::Constant(num_axes, max_acceleration);
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

    // Store the result in a flat vector: (time, positions[6], velocities[6], accelerations[6])
    std::vector<double> trajectory_flat;
    trajectory_flat.reserve((num_axes * 3 + 1) * times.size());
    for (Eigen::Index index = 0; index < times.size(); ++index) {
        const double time = times[index];
        trajectory_flat.push_back(time);
        for (int order = 0; order < 3; ++order) {
            const auto values = trajectory.eval_single(time, order);
            trajectory_flat.insert(trajectory_flat.end(), values.begin(), values.end());
        }
    }

    return trajectory_flat;
}
