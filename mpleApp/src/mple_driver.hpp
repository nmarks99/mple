#pragma once

#include <atomic>
#include <optional>
#include <pvxs/iochooks.h>
#include <pvxs/nt.h>
#include <pvxs/server.h>
#include <pvxs/sharedpv.h>

#include "mple_adapter_registry.hpp"
#include "collisions.hpp"

class MpleDriver {
  public:
    MpleDriver(const std::string& adapter_name, const std::string& prefix, const std::string& obstacles_file);

  private:
    MpleAdapter* adapter_;
    CollisionWorld collision_world_ {};

    pvxs::Value status_value_;
    pvxs::server::SharedPV status_pv_ = pvxs::server::SharedPV::buildReadonly();

    // pvxs::Value running_value_;
    // pvxs::server::SharedPV running_pv_ = pvxs::server::SharedPV::buildReadonly();

    pvxs::Value stop_value_;
    std::atomic<bool> should_stop_ = false;
    pvxs::server::SharedPV stop_pv_ = pvxs::server::SharedPV::buildMailbox();

    pvxs::server::SharedPV plan_pv_ = pvxs::server::SharedPV::buildReadonly();
    pvxs::server::SharedPV validate_pv_ = pvxs::server::SharedPV::buildReadonly();
    pvxs::server::SharedPV execute_pv_ = pvxs::server::SharedPV::buildReadonly();
    pvxs::server::SharedPV ik_pv_ = pvxs::server::SharedPV::buildReadonly();

    struct ValidateRequest {
        ValidateRequest(pvxs::Value& value, int njoints)
            : num_joints(njoints),
              joint_waypoints_flat(value["joint_waypoints"].as<pvxs::shared_array<const double>>()),
              cartesian_waypoints_flat(value["cartesian_waypoints"].as<pvxs::shared_array<const double>>()) {

            // Invalid if both full or both empty
            if (joint_waypoints_flat.empty() == cartesian_waypoints_flat.empty()) {
                printf("Only one of 'joint_waypoints' or 'cartesian_waypoints' must be provided\n");
                request_valid_ = false;
                return;
            }

            if (!joint_waypoints_flat.empty()) {
                if (joint_waypoints_flat.size() % num_joints != 0) {
                    printf("joint_waypoints shape does not match number of joints\n");
                    request_valid_ = false;
                    return;
                }
                // reshape to 2d vector
                joint_waypoints.reserve(joint_waypoints_flat.size() / num_joints);
                for (size_t i = 0; i < joint_waypoints_flat.size(); i += num_joints) {
                    joint_waypoints.emplace_back(joint_waypoints_flat.begin() + i,
                                                 joint_waypoints_flat.begin() + i + num_joints);
                }
            }

            else if (!cartesian_waypoints_flat.empty()) {
                if (cartesian_waypoints_flat.size() % 6 != 0) {
                    printf("cartesian_waypoints is not reshapeble to vectors of length 6 (x,y,z,rx,ry,rz)\n");
                    request_valid_ = false;
                    return;
                }
                // reshape to 2d vector
                cartesian_waypoints.reserve(cartesian_waypoints_flat.size() / 6);
                for (size_t i = 0; i < cartesian_waypoints_flat.size(); i += 6) {
                    cartesian_waypoints.emplace_back(cartesian_waypoints_flat.begin() + i,
                                                 cartesian_waypoints_flat.begin() + i + 6);
                }
            }

        }

        static std::optional<ValidateRequest> build(pvxs::Value& value, MpleAdapter* adapter) {
            try {
                auto validate_req = ValidateRequest(value, adapter->get_num_joints());
                if (validate_req.request_valid_) {
                    return validate_req;
                } else {
                    return std::nullopt;
                }
            } catch (...) {
            }
            return std::nullopt;
        }

        int num_joints = 0;
        pvxs::shared_array<const double> joint_waypoints_flat = {};
        std::vector<std::vector<double>> joint_waypoints = {};
        pvxs::shared_array<const double> cartesian_waypoints_flat = {};
        std::vector<std::vector<double>> cartesian_waypoints = {};

      private:
        bool request_valid_ = true;
    };

    struct ValidateResponse {
        bool valid = false;
        std::string description = "";

        pvxs::TypeDef response() {
            return pvxs::TypeDef(pvxs::TypeCode::Struct, {
                pvxs::members::Bool("valid"),
                pvxs::members::String("description"),
            });
        }

        pvxs::Value to_value() {
            auto value = response().create();
            value["valid"] = valid;
            value["description"] = description;
            return value;
        }
    };


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

        static std::optional<ExecuteRequest> build(pvxs::Value& value, MpleAdapter* adapter) {
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


    struct IKRequest {
      public:
        IKRequest(pvxs::Value& value)
            : cartesian_waypoints_flat(value["cartesian_waypoints"].as<pvxs::shared_array<const double>>()) {

                if (cartesian_waypoints_flat.size() % 6 != 0) {
                    printf("cartesian_waypoints is not reshapeble to vectors of length 6 (x,y,z,rx,ry,rz)\n");
                    request_valid_ = false;
                    return;
                }

                // reshape to 2d vector
                cartesian_waypoints.reserve(cartesian_waypoints_flat.size() / 6);
                for (size_t i = 0; i < cartesian_waypoints_flat.size(); i += 6) {
                    cartesian_waypoints.emplace_back(cartesian_waypoints_flat.begin() + i,
                                                 cartesian_waypoints_flat.begin() + i + 6);
                }
        }

        pvxs::shared_array<const double> cartesian_waypoints_flat = {};
        std::vector<std::vector<double>> cartesian_waypoints = {};

        static std::optional<IKRequest> build(pvxs::Value& value) {
            try {
                auto ik_req = IKRequest(value);
                if (ik_req.request_valid_) {
                    return ik_req;
                } else {
                    return std::nullopt;
                }
            } catch (...) {
            }
            return std::nullopt;
        }

      private:
        bool request_valid_ = true;
    };

    struct IKResponse {
        bool valid = false;
        std::string description = "";
        pvxs::shared_array<double> joint_angles_flat = {};

        pvxs::TypeDef response() {
            return pvxs::TypeDef(pvxs::TypeCode::Struct, {
                pvxs::members::Bool("valid"),
                pvxs::members::String("description"),
                pvxs::members::Float64A("joint_angles"),
            });
        }

        pvxs::Value to_value() {
            auto value = response().create();
            value["valid"] = valid;
            value["description"] = description;
            value["joint_angles"] = joint_angles_flat.freeze();
            return value;
        }
    };

    void stop_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& put_value);

    void validate_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value);

    void plan_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value);

    void execute_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value);

    void ik_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value);
};
