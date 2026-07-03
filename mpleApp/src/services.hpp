#pragma once
#include <cassert>
#include <optional>
#include <future>
#include <pvxs/sharedpv.h>
#include "mple_driver.hpp"
#include "robot_adapter_registry.hpp"
#include "trajectory.hpp"

namespace service {

namespace validate {

struct Request {
    Request(pvxs::Value& value) :
        joint_waypoints(value["joint_waypoints"].as<pvxs::shared_array<const double>>()),
        cartesian_waypoints(value["cartesian_waypoints"].as<pvxs::shared_array<const double>>()),
        resolution(value["resolution"].as<double>())
    {
        assert(value["resolution"].valid());
    }

    // TODO: these needs to be 2d arrays
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

inline void callback(MpleDriver* drv, RobotAdapter* adapter,
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
    printf("joint_waypoints.size() = %ld\n", request->joint_waypoints.size());
    printf("cartesian_waypoints.size() = %ld\n", request->cartesian_waypoints.size());
    printf("Got Resolution = %lf\n", request->resolution);
    op->reply(response.to_value());
}

}

namespace execute {

// request from the RPC client
struct Request {
    Request(pvxs::Value& value, int njoints) :
        num_joints(njoints),
        joint_waypoints_flat(value["joint_waypoints"].as<pvxs::shared_array<const double>>()),
        speed(value["speed"].as<double>()),
        acceleration(value["acceleration"].as<double>())
    {
        // TODO: this crashes the IOC???
        assert(joint_waypoints_flat.size() > 0 && "joint_waypoints empty");
        assert(joint_waypoints_flat.size() % num_joints == 0 && "joint_waypoints does not match number of joints");

        // reshape to 2d vector
        joint_waypoints.reserve(joint_waypoints_flat.size() / 2);
        for (size_t i = 0; i < joint_waypoints_flat.size(); i += 2) {
            joint_waypoints.emplace_back(joint_waypoints_flat.begin() + i, joint_waypoints_flat.begin() + i + 2);
        }
    }

    int num_joints = 0;
    pvxs::shared_array<const double> joint_waypoints_flat = {};
    std::vector<std::vector<double>> joint_waypoints = {};
    double speed = 0.0;
    double acceleration = 0.0;
};

inline std::optional<Request> build_request(pvxs::Value& value, RobotAdapter* adapter) {
    try {
        return Request(value, adapter->get_num_joints());
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

inline void callback(MpleDriver* drv, RobotAdapter* adapter,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
    Response response;
    auto request = build_request(request_value, adapter);

    if (!request) {
        response.error_code = -1;
        response.description = "Invalid request";
        op->reply(response.to_value());
        return;
    }

    std::thread exec_thread([adapter, req=std::move(request.value()), drv](){
        drv->lock();
        drv->setIntegerParam(drv->idx_mt_active_, 1);
        drv->unlock();
        adapter->execute_trajectory(req.joint_waypoints, req.speed, req.acceleration);
        drv->lock();
        drv->setIntegerParam(drv->idx_mt_active_, 0);
        drv->unlock();
    });
    exec_thread.detach();

    // Reply right away saying we started the process
    response.error_code = 0;
    response.description = "Execution started";
    op->reply(response.to_value());
    return;
}

}

}
