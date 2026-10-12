#pragma once
#include <cmath>
#include <vector>
#include <iostream>

struct TrajectoryParams {
    double max_speed;
    double max_accel;
    double dt;
};

namespace detail {

inline constexpr double cubic_time_scaling(double Tf, double t) {
    const double timeratio = 1.0*t / Tf;
    const double st = 3 * timeratio * timeratio  - 2 * timeratio*timeratio*timeratio;
    return st;
}

inline constexpr double quintic_time_scaling(double Tf, double t) {
    const double timeratio = 1.0*t / Tf;
    const double st = 10 * std::pow(timeratio, 3) - 15 * std::pow(timeratio, 4) + 6 * std::pow(timeratio, 5);
    return st;
}

inline std::pair<double, int> compute_total_time_and_steps(const std::vector<double>& q0,
                                                const std::vector<double>& q1,
                                                TrajectoryParams params) {
    constexpr double QUINTIC_PEAK_VELOCITY = 15.0 / 8.0;
    const double QUINTIC_PEAK_ACCELERATION = 10.0 / std::sqrt(3.0);
    // constexpr double dt = 1.0 / 500.0; // servo period = 1/500Hz

    // Maximum distance any joint travels over the path
    double max_distance = 0.0;
    for (size_t i = 0; i < q0.size(); i++) {
        double d = std::abs(q0[i] - q1[i]);
        if (d > max_distance) {
            max_distance = d;
        }
    }

    // compute velocity limited time, and acceleration limited time
    double velocity_time = QUINTIC_PEAK_VELOCITY * max_distance / params.max_speed;
    double acceleration_time = std::sqrt(QUINTIC_PEAK_ACCELERATION * max_distance / params.max_accel);
    double min_time = std::max(velocity_time, acceleration_time);

    int steps = std::max(1, static_cast<int>(std::ceil(min_time/params.dt)));
    double T = steps * params.dt;
    int N = steps + 1;

    return {T, N};
}
}

// Creates a time-scaled trajectory of joint states with the given
// total time and number of steps
inline std::vector<std::vector<double>> joint_trajectory(const std::vector<double>& q0,
                                                  const std::vector<double>& q1, TrajectoryParams params) {

    std::vector<std::vector<double>> traj;

    auto [Tf, N] = detail::compute_total_time_and_steps(q0, q1, params);

    double timegap = Tf / (N - 1);
    double st = 0.0;
    for (int i = 0; i < N; ++i) {
        st = detail::quintic_time_scaling(Tf, timegap*i);
        std::vector<double> row;
        for (int j = 0; j < 6; ++j) {
            row.push_back(st * q1[j] + (1 - st)*q0[j]);
        }
        traj.push_back(row);
    }

    return traj;

}

// int main() {
//
    // // User inputs
    // double speed = 1.0; // rad/s
    // std::vector<double> q0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // std::vector<double> q1 = {0.0, 1.57, 0.7, 0.7, 0.0, 0.0};
//
    // TrajectoryParams params = {
        // .max_speed = 2.0,
        // .max_accel = 2.0,
        // .dt = 0.002
    // };
    // auto traj = joint_trajectory(q0, q1, params);
//
    // std::cout << "Size: (" <<  traj.size() << "," << traj[0].size() << ")\n";
// }
