#pragma once
#include <vector>

struct TrajectoryPoint {
    double time;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
};

using Trajectory = std::vector<TrajectoryPoint>;

Trajectory build_trajectory(const std::vector<std::vector<double>>& states, double max_velocity,
                            double max_acceleration);
