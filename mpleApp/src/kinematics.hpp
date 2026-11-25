#pragma once
#include "system_model.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/math.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace mple {

std::vector<double> forward_kinematics(const SystemModel& sys, const std::vector<double>& state);

// TODO:
// std::vector<double> inverse_kinematics(SystemModel& sys, const std::vector<double>& pose);

} // namespace mple
