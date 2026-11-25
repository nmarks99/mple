#pragma once
#include <pvxs/data.h>
#include <string>
#include <optional>
#include <vector>
#include "planner.hpp"

namespace mple {

struct PlanRequest {
    pvxs::shared_array<const double> start = {};
    pvxs::shared_array<const double> goal = {};
    PlannerAlgorithm planner = PlannerAlgorithm::RRTConnect;
    double planning_time = 1.0;
    double resolution = DEFAULT_CHECK_RESOLUTION;
};

struct PlanResponse {
    bool valid = false;
    std::string description = "";
    pvxs::shared_array<const double> trajectory;
};

std::optional<PlanRequest> build_plan_request(const pvxs::Value& value, size_t num_states);

pvxs::Value to_value(const PlanResponse& response);

struct ValidateRequest {
    std::vector<std::vector<double>> joint_waypoints;
    std::vector<std::vector<double>> cartesian_waypoints;
    double resolution = DEFAULT_CHECK_RESOLUTION;
};

struct ValidateResponse {
    bool valid = false;
    std::string description;
};

std::optional<ValidateRequest> build_validate_request(const pvxs::Value& value, size_t num_states);

pvxs::Value to_value(const ValidateResponse& response);


struct ExecuteRequest {
    std::vector<std::vector<double>> waypoints;
    double speed = 0.0;
    double acceleration = 0.0;
    double timeout = 300.0;
};

struct ExecuteResponse {
    int error_code = 0;
    std::string description;
};

std::optional<ExecuteRequest> build_execute_request(const pvxs::Value& value, size_t num_states);

pvxs::Value to_value(const ExecuteResponse& response);

struct FKRequest {
    std::vector<std::vector<double>> joints;
};

struct FKResponse {
    bool valid = false;
    std::string description;
    pvxs::shared_array<const double> poses;
};

std::optional<FKRequest> build_fk_request(const pvxs::Value& value, size_t num_states);

pvxs::Value to_value(const FKResponse& response);

struct IKRequest {
    std::vector<std::vector<double>> cartesian_waypoints;
};

struct IKResponse {
    bool valid = false;
    std::string description;
    pvxs::shared_array<const double> joint_angles;
};

std::optional<IKRequest> build_ik_request(const pvxs::Value& value);

pvxs::Value to_value(const IKResponse& response);

}
