#include <cmath>
#include "rpc.hpp"

namespace mple {

namespace {

std::vector<std::vector<double>> reshape(const pvxs::shared_array<const double>& flat, size_t width) {
    std::vector<std::vector<double>> out;
    out.reserve(flat.size() / width);
    for (size_t i = 0; i < flat.size(); i += width) {
        out.emplace_back(flat.begin() + i, flat.begin() + i + width);
    }
    return out;
}

} // namespace

std::optional<PlanRequest> build_plan_request(const pvxs::Value& value, size_t num_states) {

    PlanRequest req{};

    req.start = value["start"].as<pvxs::shared_array<const double>>();
    req.goal = value["goal"].as<pvxs::shared_array<const double>>();

    const auto goal_size = req.goal.size();
    const auto start_size = req.start.size();
    if (goal_size != num_states) {
        printf("Goal state must be equal to number of states: %zu\n", num_states);
        return std::nullopt;
    }
    if (start_size != 0 && start_size != goal_size) {
        printf("Start and goal must both be equal to number of states: %zu\n", num_states);
        return std::nullopt;
    }

    if (auto field = value["planner"]; field.valid()) {
        const auto name = field.as<std::string>();
        if (!name.empty()) {
            auto parsed = planner_from_string(name);
            if (!parsed) {
                printf("Unknown planner: %s\n", name.c_str());
                return std::nullopt;
            }
            req.planner = parsed.value();
        }
    }

    if (auto field = value["planning_time"]; field.valid()) {
        req.planning_time = field.as<double>();
    }
    if (auto field = value["resolution"]; field.valid()) {
        req.resolution = field.as<double>();
    }

    return req;
}

pvxs::Value to_value(const PlanResponse& response) {
    static const pvxs::TypeDef td = pvxs::TypeDef(pvxs::TypeCode::Struct, {
        pvxs::members::Bool("valid"),
        pvxs::members::String("description"),
        pvxs::members::Float64A("trajectory"),
    });

    auto value = td.create();
    value["valid"] = response.valid;
    value["description"] = response.description;
    value["trajectory"] = response.trajectory;
    return value;
}

std::optional<ValidateRequest> build_validate_request(const pvxs::Value& value, size_t num_states) {

    ValidateRequest req{};

    const auto joint_flat = value["joint_waypoints"].as<pvxs::shared_array<const double>>();
    const auto cartesian_flat = value["cartesian_waypoints"].as<pvxs::shared_array<const double>>();

    if (joint_flat.empty() == cartesian_flat.empty()) {
        printf("Only one of 'joint_waypoints' or 'cartesian_waypoints' must be provided\n");
        return std::nullopt;
    }

    if (!joint_flat.empty()) {
        if (joint_flat.size() % num_states != 0) {
            printf("joint_waypoints shape does not match number of states: %zu\n", num_states);
            return std::nullopt;
        }
        req.joint_waypoints = reshape(joint_flat, num_states);
    } else {
        if (cartesian_flat.size() % 6 != 0) {
            printf("cartesian_waypoints is not reshapeble to vectors of length 6 (x,y,z,rx,ry,rz)\n");
            return std::nullopt;
        }
        req.cartesian_waypoints = reshape(cartesian_flat, 6);
    }

    if (auto field = value["resolution"]; field.valid()) {
        req.resolution = field.as<double>();
        if (!(req.resolution > 0.0) || !std::isfinite(req.resolution)) {
            printf("resolution must be finite and greater than 0\n");
            return std::nullopt;
        }
    }

    return req;
}

pvxs::Value to_value(const ValidateResponse& response) {
    static const pvxs::TypeDef td = pvxs::TypeDef(pvxs::TypeCode::Struct, {
        pvxs::members::Bool("valid"),
        pvxs::members::String("description"),
    });

    auto value = td.create();
    value["valid"] = response.valid;
    value["description"] = response.description;
    return value;
}


std::optional<ExecuteRequest> build_execute_request(const pvxs::Value& value, size_t num_states) {

    ExecuteRequest req{};

    const auto waypoints_flat = value["waypoints"].as<pvxs::shared_array<const double>>();
    req.speed = value["speed"].as<double>();
    req.acceleration = value["acceleration"].as<double>();
    req.timeout = value["timeout"].as<double>();

    if (num_states == 0) {
        printf("Number of states must be > 0\n");
        return std::nullopt;
    }
    if (waypoints_flat.size() < 2 * num_states) {
        printf("At least two waypoints are required\n");
        return std::nullopt;
    }
    if (waypoints_flat.size() % num_states != 0) {
        printf("waypoints shape does not match number of states: %zu\n", num_states);
        return std::nullopt;
    }
    req.waypoints = reshape(waypoints_flat, num_states);

    return req;
}

pvxs::Value to_value(const ExecuteResponse& response) {
    static const pvxs::TypeDef td = pvxs::TypeDef(pvxs::TypeCode::Struct, {
        pvxs::members::Int32("error_code"),
        pvxs::members::String("description"),
    });

    auto value = td.create();
    value["error_code"] = response.error_code;
    value["description"] = response.description;
    return value;
}

std::optional<FKRequest> build_fk_request(const pvxs::Value& value, size_t num_states) {

    FKRequest req{};

    const auto joints_flat = value["joints"].as<pvxs::shared_array<const double>>();
    if (joints_flat.size() % num_states != 0) {
        printf("joints is not reshapeble to vectors of length %zu (the number of states)\n", num_states);
        return std::nullopt;
    }
    req.joints = reshape(joints_flat, num_states);

    return req;
}

pvxs::Value to_value(const FKResponse& response) {
    static const pvxs::TypeDef td = pvxs::TypeDef(pvxs::TypeCode::Struct, {
        pvxs::members::Bool("valid"),
        pvxs::members::String("description"),
        pvxs::members::Float64A("poses"),
    });

    auto value = td.create();
    value["valid"] = response.valid;
    value["description"] = response.description;
    value["poses"] = response.poses;
    return value;
}

std::optional<IKRequest> build_ik_request(const pvxs::Value& value) {

    IKRequest req{};

    const auto cartesian_flat = value["cartesian_waypoints"].as<pvxs::shared_array<const double>>();
    if (cartesian_flat.size() % 6 != 0) {
        printf("cartesian_waypoints is not reshapeble to vectors of length 6 (x,y,z,rx,ry,rz)\n");
        return std::nullopt;
    }
    req.cartesian_waypoints = reshape(cartesian_flat, 6);

    return req;
}

pvxs::Value to_value(const IKResponse& response) {
    static const pvxs::TypeDef td = pvxs::TypeDef(pvxs::TypeCode::Struct, {
        pvxs::members::Bool("valid"),
        pvxs::members::String("description"),
        pvxs::members::Float64A("joint_angles"),
    });

    auto value = td.create();
    value["valid"] = response.valid;
    value["description"] = response.description;
    value["joint_angles"] = response.joint_angles;
    return value;
}

}
