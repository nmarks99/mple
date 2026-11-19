#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

enum class PlannerAlgorithm {
    RRTConnect,
    RRTstar,
    BITstar,
    ABITstar
};

struct PlanResult {
    bool valid = false;
    std::string description;
    std::vector<double> trajectory_flat;
};

using StateValidityChecker = std::function<bool(const std::vector<double>&)>;

struct PlanParams {
    std::vector<double> start;
    std::vector<double> goal;
    std::vector<double> lower_limits;
    std::vector<double> upper_limits;
    StateValidityChecker is_state_valid;
    double planning_time = 1.0;
    PlannerAlgorithm algo = PlannerAlgorithm::RRTConnect;
};

std::optional<PlannerAlgorithm> planner_from_string(const std::string& name);

PlanResult plan(const PlanParams& params);
