#pragma once
#include <vector>

#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/parametrizer/spline.hpp>
#include <toppra/parametrizer/const_accel.hpp>
#include <toppra/solver/seidel.hpp>

std::vector<double> build_trajectory(const std::vector<std::vector<double>>& joint_states,
                                     int dof, double max_velocity, double max_acceleration);
