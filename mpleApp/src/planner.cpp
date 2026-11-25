#include "planner.hpp"
#include <cmath>
#include <algorithm>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/EITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/lazyinformedtrees/BLITstar.h>
#include <ompl/geometric/planners/rrt/AORRTC.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

std::optional<PlannerAlgorithm> planner_from_string(const std::string& name) {
    if (name == "RRTConnect") {
        return PlannerAlgorithm::RRTConnect;
    }
    if (name == "RRTstar") {
        return PlannerAlgorithm::RRTstar;
    }
    if (name == "BITstar") {
        return PlannerAlgorithm::BITstar;
    }
    if (name == "ABITstar") {
        return PlannerAlgorithm::ABITstar;
    }
    if (name == "EITstar") {
        return PlannerAlgorithm::EITstar;
    }
    if (name == "AITstar") {
        return PlannerAlgorithm::AITstar;
    }
    if (name == "BLITstar") {
        return PlannerAlgorithm::BLITstar;
    }
    if (name == "AORRTC") {
        return PlannerAlgorithm::AORRTC;
    }
    if (name == "AnytimePathShortening") {
        return PlannerAlgorithm::AnytimePathShortening;
    }
    return std::nullopt;
}

PlanResult plan(const PlanParams& params) {
    PlanResult result;

    if (!(params.planning_time > 0.0)) {
        result.description = "Planning time must be greater than 0";
        return result;
    }

    if (!(params.resolution > 0.0) || !std::isfinite(params.resolution)) {
        result.description = "Resolution must be finite and greater than 0";
        return result;
    }

    if (!params.is_state_valid) {
        result.description = "State validity checker must be set";
        return result;
    }

    if (params.start.empty() || params.start.size() != params.goal.size()) {
        result.description = "Start and goal dimensions must be equal and nonzero";
        return result;
    }

    const size_t state_dimension = params.start.size();
    if (params.lower_limits.size() != state_dimension || params.upper_limits.size() != state_dimension) {
        result.description = "Joint limit dimensions must match the state dimension";
        return result;
    }

    // Set bounds from the given limits
    ompl::base::RealVectorBounds bounds(state_dimension);
    for (size_t i = 0; i < state_dimension; ++i) {
        if (!std::isfinite(params.start[i]) || !std::isfinite(params.goal[i])) {
            result.description = "Start and goal values for joint " + std::to_string(i) + " must be finite";
            return result;
        }
        // An unbounded (continuous) joint may move one turn beyond its start and goal, not more
        if (params.lower_limits[i] == -INFINITY && params.upper_limits[i] == INFINITY) {
            bounds.setLow(i, std::min(params.start[i], params.goal[i]) - 2.0 * M_PI);
            bounds.setHigh(i, std::max(params.start[i], params.goal[i]) + 2.0 * M_PI);
            continue;
        }
        if (!std::isfinite(params.lower_limits[i]) || !std::isfinite(params.upper_limits[i]) ||
            params.lower_limits[i] >= params.upper_limits[i]) {
            result.description = "Invalid joint limits for joint " + std::to_string(i);
            return result;
        }
        if (params.start[i] < params.lower_limits[i] || params.start[i] > params.upper_limits[i]) {
            result.description = "Start value for joint " + std::to_string(i) + " is outside joint limits";
            return result;
        }
        if (params.goal[i] < params.lower_limits[i] || params.goal[i] > params.upper_limits[i]) {
            result.description = "Goal value for joint " + std::to_string(i) + " is outside joint limits";
            return result;
        }
        bounds.setLow(i, params.lower_limits[i]);
        bounds.setHigh(i, params.upper_limits[i]);
    }
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(state_dimension);
    space->setBounds(bounds);
    if (params.resolution >= space->getMaximumExtent()) {
        result.description = "Resolution must be smaller than the state space extent";
        return result;
    }

    // SimpleSetup convenience class for setting up the planning problem
    ompl::geometric::SimpleSetup setup(space);

    // Set the state validity checker which is used to determine if each sampled
    // configuration in the state space is valid (i.e. no collision or limit violation)
    setup.setStateValidityChecker(
        [&params, state_dimension](const ompl::base::State* state) {
            const auto* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            return params.is_state_valid(std::vector<double>(values, values + state_dimension));
        });
    setup.getSpaceInformation()->setStateValidityCheckingResolution(params.resolution /
                                                                    space->getMaximumExtent());

    // Tell optimization based planners to optimize for path length
    setup.setOptimizationObjective(
        std::make_shared<ompl::base::PathLengthOptimizationObjective>(setup.getSpaceInformation())
    );

    // Set the requested planning algorithm
    switch (params.algo) {
        case PlannerAlgorithm::RRTConnect:
            setup.setPlanner(std::make_shared<ompl::geometric::RRTConnect>(setup.getSpaceInformation()));
            break;
        case PlannerAlgorithm::RRTstar:
            setup.setPlanner(std::make_shared<ompl::geometric::RRTstar>(setup.getSpaceInformation()));
            break;
        case PlannerAlgorithm::BITstar:
            setup.setPlanner(std::make_shared<ompl::geometric::BITstar>(setup.getSpaceInformation(), "kBITstar"));
            break;
        case PlannerAlgorithm::ABITstar:
            setup.setPlanner(std::make_shared<ompl::geometric::ABITstar>(setup.getSpaceInformation(), "kABITstar"));
            break;
        case PlannerAlgorithm::EITstar:
            setup.setPlanner(std::make_shared<ompl::geometric::EITstar>(setup.getSpaceInformation()));
            break;
        case PlannerAlgorithm::AITstar:
            setup.setPlanner(std::make_shared<ompl::geometric::AITstar>(setup.getSpaceInformation()));
            break;
        case PlannerAlgorithm::BLITstar:
            setup.setPlanner(std::make_shared<ompl::geometric::BLITstar>(setup.getSpaceInformation()));
            break;
        case PlannerAlgorithm::AORRTC:
            setup.setPlanner(std::make_shared<ompl::geometric::AORRTC>(setup.getSpaceInformation()));
            break;
        case PlannerAlgorithm::AnytimePathShortening:
            setup.setPlanner(
                std::make_shared<ompl::geometric::AnytimePathShortening>(setup.getSpaceInformation()));
            break;
    }

    // Set the start and goal states with a threshold
    ompl::base::ScopedState<> start(space);
    ompl::base::ScopedState<> goal(space);
    start = params.start;
    goal = params.goal;
    setup.setStartAndGoalStates(start, goal, 1e-9);

    // Do the solve
    constexpr double MAX_PLANNING_TIME = 30.0;
    double t_plan = std::min(params.planning_time, MAX_PLANNING_TIME);
    const ompl::base::PlannerStatus status = setup.solve(t_plan);
    if (status != ompl::base::PlannerStatus::EXACT_SOLUTION) {
        result.description = "Planner failed to find an exact solution: " + status.asString();
        return result;
    }

    // Attempt to simplify the solution with the default simplification method
    setup.simplifySolution();
    ompl::geometric::PathGeometric& path = setup.getSolutionPath();
    if (!path.check()) {
        result.description = "Planner returned an invalid path";
        return result;
    }

    // Insert states so spacing is about the validity checking resolution. Useful for trajectory
    // execution, but in move_step mode every waypoint is a stop, so skipping this may be preferable.
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
