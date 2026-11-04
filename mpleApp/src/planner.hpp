#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

struct PlanResult {
    bool valid = false;
    std::string description;
    std::vector<double> trajectory_flat;
};

inline PlanResult plan(const std::vector<double>& start_values,
                       const std::vector<double>& goal_values,
                       unsigned int num_intermediate_points) {
    PlanResult result;

    if (start_values.empty() || start_values.size() != goal_values.size()) {
        result.description = "Start and goal dimensions must be equal and nonzero";
        return result;
    }

    auto space = std::make_shared<ompl::base::CompoundStateSpace>();
    for (size_t i = 0; i < start_values.size(); ++i) {
        space->addSubspace(std::make_shared<ompl::base::SO2StateSpace>(), 1.0);
    }

    ompl::geometric::SimpleSetup setup(space);

    // TODO: this should check for collisions
    setup.setStateValidityChecker([](const ompl::base::State*) { return true; });

    setup.setPlanner(
        std::make_shared<ompl::geometric::RRTConnect>(setup.getSpaceInformation()));

    ompl::base::ScopedState<> start(space);
    ompl::base::ScopedState<> goal(space);
    auto* start_state = start->as<ompl::base::CompoundStateSpace::StateType>();
    auto* goal_state = goal->as<ompl::base::CompoundStateSpace::StateType>();
    for (size_t i = 0; i < start_values.size(); ++i) {
        start_state->as<ompl::base::SO2StateSpace::StateType>(i)->value = start_values[i];
        goal_state->as<ompl::base::SO2StateSpace::StateType>(i)->value = goal_values[i];
    }
    space->enforceBounds(start.get());
    space->enforceBounds(goal.get());
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

    const size_t total_points = static_cast<size_t>(num_intermediate_points) + 2;
    path.interpolate(total_points);
    result.trajectory_flat.resize(total_points * start_values.size());

    constexpr double two_pi = 6.28318530717958647692;
    std::vector<double> previous = start_values;
    for (size_t point_index = 0; point_index < path.getStateCount(); ++point_index) {
        const auto* state =
            path.getState(point_index)->as<ompl::base::CompoundStateSpace::StateType>();
        for (size_t state_index = 0; state_index < start_values.size(); ++state_index) {
            const double wrapped =
                state->as<ompl::base::SO2StateSpace::StateType>(state_index)->value;
            const double unwrapped =
                previous[state_index] + std::remainder(wrapped - previous[state_index], two_pi);
            result.trajectory_flat[point_index * start_values.size() + state_index] = unwrapped;
            previous[state_index] = unwrapped;
        }
    }

    result.valid = true;
    result.description = "Success";
    return result;
}
