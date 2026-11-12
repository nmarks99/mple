#pragma once
#include "mple_system_model.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/math.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>

std::vector<double> forward_kinematics(const MpleSystemModel& sys, const std::vector<double>& state);

// TODO:
// std::vector<double> inverse_kinematics(MpleSystemModel& sys, const std::vector<double>& pose);
