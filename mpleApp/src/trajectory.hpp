#pragma once
#include <vector>

std::vector<double> build_trajectory(const std::vector<std::vector<double>>& joint_states,
                                     unsigned int num_axes, double max_velocity, double max_acceleration);
