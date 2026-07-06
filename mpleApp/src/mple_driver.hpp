#pragma once
#include "asynPortDriver.h"

#include <optional>
#include <future>
#include <pvxs/iochooks.h>
#include <pvxs/nt.h>
#include <pvxs/server.h>
#include <pvxs/sharedpv.h>

#include "robot_adapter_registry.hpp"

struct ExecuteRequest {
    ExecuteRequest(pvxs::Value& value, int njoints)
        : num_joints(njoints),
          joint_waypoints_flat(value["joint_waypoints"].as<pvxs::shared_array<const double>>()),
          speed(value["speed"].as<double>()), acceleration(value["acceleration"].as<double>()) {
        // TODO: assert throwing here crashes the IOC???
        assert(joint_waypoints_flat.size() > 0 && "joint_waypoints empty");
        assert(joint_waypoints_flat.size() % num_joints == 0 &&
               "joint_waypoints does not match number of joints");

        // reshape to 2d vector
        joint_waypoints.reserve(joint_waypoints_flat.size() / 2);
        for (size_t i = 0; i < joint_waypoints_flat.size(); i += num_joints) {
            joint_waypoints.emplace_back(joint_waypoints_flat.begin() + i,
                                         joint_waypoints_flat.begin() + i + num_joints);
        }
    }

    static std::optional<ExecuteRequest> build(pvxs::Value& value, RobotAdapter* adapter) {
        try {
            return ExecuteRequest(value, adapter->get_num_joints());
        } catch (...) {
        }
        return std::nullopt;
    }

    int num_joints = 0;
    pvxs::shared_array<const double> joint_waypoints_flat = {};
    std::vector<std::vector<double>> joint_waypoints = {};
    double speed = 0.0;
    double acceleration = 0.0;
};


struct ExecuteResponse {
    int error_code = 0;
    std::string description = "";

    pvxs::TypeDef response() const {
        return pvxs::TypeDef(pvxs::TypeCode::Struct, {
            pvxs::members::Int32("error_code"),
            pvxs::members::String("description"),
        });
    }

    pvxs::Value to_value() {
        auto value = response().create();
        value["error_code"] = error_code;
        value["description"] = description;
        return value;
    }
};

class MpleDriver : public asynPortDriver {
  public:
    MpleDriver(const char* port, const char* script);
    virtual void poll(void);
    int idx_mt_active_;
    int idx_mt_error_;
    std::vector<std::future<void>> tasks;

  private:
    asynUser* pasynUserDriver_;
    pvxs::server::SharedPV plan_pv_ = pvxs::server::SharedPV::buildReadonly();
    pvxs::server::SharedPV validate_pv_ = pvxs::server::SharedPV::buildReadonly();
    pvxs::server::SharedPV execute_pv_ = pvxs::server::SharedPV::buildReadonly();

    void execute_callback(RobotAdapter* adapter,
                          std::unique_ptr<pvxs::server::ExecOp>&& op, pvxs::Value&& request_value) {
        ExecuteResponse response;
        auto request = ExecuteRequest::build(request_value, adapter);

        if (!request) {
            response.error_code = -1;
            response.description = "Invalid request";
            op->reply(response.to_value());
            return;
        }

        std::thread exec_thread([adapter, req = std::move(request.value()), this]() {
            lock();
            setIntegerParam(idx_mt_active_, 1);
            unlock();
            adapter->execute_trajectory(req.joint_waypoints, req.speed, req.acceleration);
            lock();
            setIntegerParam(idx_mt_active_, 0);
            unlock();
        });
        exec_thread.detach();

        // Reply right away saying we started the process
        response.error_code = 0;
        response.description = "Execution started";
        op->reply(response.to_value());
        return;
    }
};
