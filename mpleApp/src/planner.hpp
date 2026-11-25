#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

enum class PlannerAlgorithm {
    RRTConnect,
    RRTstar,
    BITstar,
    ABITstar,
    EITstar,
    AITstar,
    BLITstar,
    AORRTC,
    AnytimePathShortening
};

struct PlanResult {
    bool valid = false;
    std::string description;
    std::vector<double> trajectory_flat;
};

// Joint-space distance (Euclidean, radians or meters) between collision-checked states
inline constexpr double DEFAULT_CHECK_RESOLUTION = 0.01;

// AnytimePathShortening calls this from several threads at once
using StateValidityChecker = std::function<bool(const std::vector<double>&)>;

struct PlanParams {
    std::vector<double> start;
    std::vector<double> goal;
    // -inf/+inf marks an unbounded (continuous) joint
    std::vector<double> lower_limits;
    std::vector<double> upper_limits;
    StateValidityChecker is_state_valid;
    double planning_time = 1.0;
    double resolution = DEFAULT_CHECK_RESOLUTION;
    PlannerAlgorithm algo = PlannerAlgorithm::RRTConnect;
};

std::optional<PlannerAlgorithm> planner_from_string(const std::string& name);

PlanResult plan(const PlanParams& params);
