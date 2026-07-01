#pragma once
#include <cassert>
#include <optional>
#include <pvxs/sharedpv.h>
#include "robot_adapter_registry.hpp"

namespace rpc {

namespace validate {

// request from the RPC client
struct Request {
    Request(pvxs::Value& value) :
        joint_waypoints(value["joint_waypoints"].as<pvxs::shared_array<const double>>()),
        cartesian_waypoints(value["cartesian_waypoints"].as<pvxs::shared_array<const double>>()),
        resolution(value["resolution"].as<double>())
    {
        assert(value["resolution"].valid());
    }
    pvxs::shared_array<const double> joint_waypoints = {};
    pvxs::shared_array<const double> cartesian_waypoints = {};
    double resolution = 0.0;
};

inline std::optional<Request> build_request(pvxs::Value& value) {
    try {
        return Request(value);
    } catch (...) {
    }
    return std::nullopt;
}

inline pvxs::TypeDef response() {
    return pvxs::TypeDef(pvxs::TypeCode::Struct, {
        pvxs::members::Bool("valid"),
        pvxs::members::String("description"),
    });
}

struct Response {
    bool valid = false;
    std::string description = "";

    pvxs::Value to_value() {
        auto value = response().create();
        value["valid"] = valid;
        value["description"] = description;
        return value;
    }
};

inline void handler(pvxs::server::SharedPV& pv,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
    Response response;
    auto request = build_request(request_value);

    if (!request) {
        response.valid = false;
        response.description = "Invalid request";
        op->reply(response.to_value());
        return;
    }

    response.valid = true;
    response.description = "Success";
    // for (auto& v : request->joint_waypoints) {
        // printf("%lf,", v);
    // }
    // printf("\n");
    printf("joint_waypoints.size() = %ld\n", request->joint_waypoints.size());
    printf("cartesian_waypoints.size() = %ld\n", request->cartesian_waypoints.size());
    printf("Got Resolution = %lf\n", request->resolution);
    op->reply(response.to_value());
}

}

namespace execute {

// request from the RPC client
struct Request {
    Request(pvxs::Value& value) :
        joint_waypoints(value["joint_waypoints"].as<pvxs::shared_array<const double>>()),
        speeds(value["speeds"].as<pvxs::shared_array<const double>>()),
        accelerations(value["accelerations"].as<pvxs::shared_array<const double>>())
    {
        assert(joint_waypoints.size() > 0);
    }
    pvxs::shared_array<const double> joint_waypoints = {};
    pvxs::shared_array<const double> speeds = {};
    pvxs::shared_array<const double> accelerations = {};
};

inline std::optional<Request> build_request(pvxs::Value& value) {
    try {
        return Request(value);
    } catch (...) {}
    return std::nullopt;
}

inline pvxs::TypeDef response() {
    return pvxs::TypeDef(pvxs::TypeCode::Struct, {
        pvxs::members::Int32("error_code"),
        pvxs::members::String("description"),
    });
}

struct Response {
    int error_code = 0;
    std::string description = "";

    pvxs::Value to_value() {
        auto value = response().create();
        value["error_code"] = error_code;
        value["description"] = description;
        return value;
    }
};

inline void handler(pvxs::server::SharedPV& pv,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
    Response response;
    auto request = build_request(request_value);

    if (!request) {
        response.error_code = -1;
        response.description = "Invalid request";
        op->reply(response.to_value());
        return;
    }

    response.error_code = 0;
    response.description = "Execution started successfully";
    // for (auto& v : request->joint_waypoints) {
        // printf("%lf,", v);
    // }
    // printf("\n");
    printf("joint_waypoints.size() = %ld\n", request->joint_waypoints.size());
    printf("speeds.size() = %ld\n", request->speeds.size());
    printf("accelerations.size() = %ld\n", request->accelerations.size());
    printf("call adapter.execute_trajectory(joint_waypoints, speeds, accelerations)\n");
    op->reply(response.to_value());
}

}

}
