#include "mple/trajectory.hpp"

#include <algorithm>
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

namespace mple {

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

    // Store waypoints in toppra::Vectors with path parameter s as the cumulative joint-space
    // distance. Waypoints that coincide with the previous one are dropped so s strictly increases.
    constexpr double MIN_SEGMENT_LENGTH = 1e-9;
    toppra::Vectors waypoints;
    std::vector<double> s_values;
    for (const auto& wp : states) {
        if (wp.size() != state_size) {
            throw std::invalid_argument("All waypoints must have the same size");
        }
        const toppra::Vector q = Eigen::Map<const toppra::Vector>(wp.data(), wp.size());
        if (waypoints.empty()) {
            s_values.push_back(0.0);
        } else {
            const double length = (q - waypoints.back()).norm();
            if (length <= MIN_SEGMENT_LENGTH) {
                continue;
            }
            s_values.push_back(s_values.back() + length);
        }
        waypoints.push_back(q);
    }
    if (waypoints.size() < 2) {
        throw std::invalid_argument("At least two distinct waypoints are required");
    }
    const toppra::Vector s_vec = Eigen::Map<const toppra::Vector>(s_values.data(), s_values.size());

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

Trajectory build_step_trajectory(const std::vector<std::vector<double>>& states, double max_velocity,
                                 double max_acceleration) {

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

    Trajectory traj{};
    traj.reserve(states.size() - 1);
    for (size_t k = 1; k < states.size(); ++k) {
        if (states[k].size() != state_size || states[k - 1].size() != state_size) {
            throw std::invalid_argument("All waypoints must have the same size");
        }

        std::vector<double> dist(state_size);
        double max_dist = 0.0;
        for (size_t i = 0; i < state_size; ++i) {
            dist[i] = std::abs(states[k][i] - states[k - 1][i]);
            max_dist = std::max(max_dist, dist[i]);
        }

        TrajectoryPoint pt{};
        pt.position = states[k];
        pt.velocity.assign(state_size, 0.0);
        pt.acceleration.assign(state_size, 0.0);

        if (max_dist > 0.0) {
            // Time-scale the joint with the largest distance, others follow the same profile
            double t_acc = 0.0;
            if (max_dist >= max_velocity * max_velocity / max_acceleration) {
                t_acc = max_velocity / max_acceleration;
                pt.time = max_dist / max_velocity + t_acc;
            } else {
                t_acc = std::sqrt(max_dist / max_acceleration);
                pt.time = 2.0 * t_acc;
            }
            const double t_cruise_end = pt.time - t_acc;
            for (size_t i = 0; i < state_size; ++i) {
                pt.velocity[i] = dist[i] / t_cruise_end;
                pt.acceleration[i] = pt.velocity[i] / t_acc;
            }
        }

        traj.push_back(std::move(pt));
    }

    return traj;
}

} // namespace mple
