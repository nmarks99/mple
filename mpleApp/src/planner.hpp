#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

struct PlanResult {
    bool valid = false;
    std::string description;
    std::vector<double> trajectory_flat;
};

using StateValidityChecker = std::function<bool(const std::vector<double>&)>;

inline PlanResult plan(const std::vector<double>& start_values,
                       const std::vector<double>& goal_values,
                       const std::vector<double>& lower_limits,
                       const std::vector<double>& upper_limits,
                       const StateValidityChecker& is_state_valid) {
    PlanResult result;

    if (start_values.empty() || start_values.size() != goal_values.size()) {
        result.description = "Start and goal dimensions must be equal and nonzero";
        return result;
    }

    const size_t state_dimension = start_values.size();
    if (lower_limits.size() != state_dimension || upper_limits.size() != state_dimension) {
        result.description = "Joint limit dimensions must match the state dimension";
        return result;
    }

    ompl::base::RealVectorBounds bounds(state_dimension);
    for (size_t i = 0; i < state_dimension; ++i) {
        if (!std::isfinite(lower_limits[i]) || !std::isfinite(upper_limits[i]) ||
            lower_limits[i] >= upper_limits[i]) {
            result.description = "Invalid joint limits for joint " + std::to_string(i);
            return result;
        }
        if (start_values[i] < lower_limits[i] || start_values[i] > upper_limits[i]) {
            result.description = "Start value for joint " + std::to_string(i) + " is outside joint limits";
            return result;
        }
        if (goal_values[i] < lower_limits[i] || goal_values[i] > upper_limits[i]) {
            result.description = "Goal value for joint " + std::to_string(i) + " is outside joint limits";
            return result;
        }
        bounds.setLow(i, lower_limits[i]);
        bounds.setHigh(i, upper_limits[i]);
    }

    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(state_dimension);
    space->setBounds(bounds);

    ompl::geometric::SimpleSetup setup(space);

    setup.setStateValidityChecker(
        [&is_state_valid, state_dimension](const ompl::base::State* state) {
            const auto* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            return is_state_valid(std::vector<double>(values, values + state_dimension));
        });
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    setup.setPlanner(
        std::make_shared<ompl::geometric::RRTConnect>(setup.getSpaceInformation()));

    ompl::base::ScopedState<> start(space);
    ompl::base::ScopedState<> goal(space);
    for (size_t i = 0; i < state_dimension; ++i) {
        start[i] = start_values[i];
        goal[i] = goal_values[i];
    }
    setup.setStartAndGoalStates(start, goal, 1e-9);

    const ompl::base::PlannerStatus status = setup.solve(1.0);
    if (status != ompl::base::PlannerStatus::EXACT_SOLUTION) {
        result.description = "Failed to find an exact path: " + status.asString();
        return result;
    }

    setup.simplifySolution();
    ompl::geometric::PathGeometric& path = setup.getSolutionPath();
    if (!path.check()) {
        result.description = "OMPL returned an invalid path";
        return result;
    }

    path.interpolate();
    result.trajectory_flat.resize(path.getStateCount() * state_dimension);
    for (size_t point_index = 0; point_index < path.getStateCount(); ++point_index) {
        const auto* values =
            path.getState(point_index)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        for (size_t state_index = 0; state_index < state_dimension; ++state_index) {
            result.trajectory_flat[point_index * state_dimension + state_index] = values[state_index];
        }
    }

    result.valid = true;
    result.description = "Success";
    return result;
}
