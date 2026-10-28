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

    enum class ExecutionState {
        Idle,
        Running,
        StopRequested,
    };

    pvxs::Value stop_value_;
    std::atomic<ExecutionState> execution_state_ = ExecutionState::Idle;
    pvxs::server::SharedPV stop_pv_ = pvxs::server::SharedPV::buildMailbox();

    pvxs::server::SharedPV plan_pv_ = pvxs::server::SharedPV::buildReadonly();
    pvxs::server::SharedPV validate_pv_ = pvxs::server::SharedPV::buildReadonly();
    pvxs::server::SharedPV execute_pv_ = pvxs::server::SharedPV::buildReadonly();
    pvxs::server::SharedPV ik_pv_ = pvxs::server::SharedPV::buildReadonly();
    pvxs::server::SharedPV fk_pv_ = pvxs::server::SharedPV::buildReadonly();

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
                auto validate_req = ValidateRequest(value, adapter->num_axes());
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
              waypoints_flat(value["waypoints"].as<pvxs::shared_array<const double>>()),
              speed(value["speed"].as<double>()),
              acceleration(value["acceleration"].as<double>()),
              timeout(value["timeout"].as<double>())
        {

            if (num_joints <= 0) {
                throw std::invalid_argument("Number of axes must be > 0");
            }
            if (waypoints_flat.size() < 2 * static_cast<size_t>(num_joints)) {
                throw std::invalid_argument("At least two waypoints are required");
            }
            if (waypoints_flat.size() % num_joints != 0) {
                throw std::invalid_argument(
                    "Waypoint data does not match the number of axes");
            }

            // reshape to 2d vector
            waypoints.reserve(waypoints_flat.size() / num_joints);
            for (size_t i = 0; i < waypoints_flat.size(); i += num_joints) {
                waypoints.emplace_back(waypoints_flat.begin() + i,
                                             waypoints_flat.begin() + i + num_joints);
            }
        }

        static std::optional<ExecuteRequest> build(pvxs::Value& value, MpleAdapter* adapter) {
            try {
                return ExecuteRequest(value, adapter->num_axes());
            } catch (...) {
            }
            return std::nullopt;
        }

        int num_joints = 0;
        pvxs::shared_array<const double> waypoints_flat = {};
        std::vector<std::vector<double>> waypoints = {};
        double speed = 0.0;
        double acceleration = 0.0;
        double timeout = 300.0;
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


    struct PlanRequest {
      public:
        PlanRequest(pvxs::Value& value, unsigned int num_joints):
            start(value["start"].as<pvxs::shared_array<const double>>()),
            goal(value["goal"].as<pvxs::shared_array<const double>>()),
            num_points(value["num_points"].as<unsigned int>()) {
            const size_t start_size = start.size();
            const size_t goal_size = goal.size();
            if (goal_size != num_joints) {
                printf("Goal state must both be equal to number of joints: %d\n", num_joints);
                request_valid_ = false;
                return;
            }
            if (start_size != 0 && start_size != goal_size) {
                printf("start and goal must both be equal to number of joints: %d\n", num_joints);
                request_valid_ = false;
                return;
            }
        }

        pvxs::shared_array<const double> start = {};
        pvxs::shared_array<const double> goal = {};
        unsigned int num_points = 0;

        static std::optional<PlanRequest> build(pvxs::Value& value, unsigned int num_joints) {
            try {
                auto plan_req = PlanRequest(value, num_joints);
                if (plan_req.request_valid_) {
                    return plan_req;
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

    struct PlanResponse {
        bool valid = false;
        std::string description = "";
        pvxs::shared_array<double> trajectory_flat = {};

        pvxs::TypeDef response() {
            return pvxs::TypeDef(pvxs::TypeCode::Struct, {
                pvxs::members::Bool("valid"),
                pvxs::members::String("description"),
                pvxs::members::Float64A("trajectory"),
            });
        }

        pvxs::Value to_value() {
            auto value = response().create();
            value["valid"] = valid;
            value["description"] = description;
            value["trajectory"] = trajectory_flat.freeze();
            return value;
        }
    };


    struct FKRequest {
      public:
        FKRequest(pvxs::Value& value, unsigned int num_joints)
            : joints_flat(value["joints"].as<pvxs::shared_array<const double>>()) {

                if (joints_flat.size() % num_joints != 0) {
                    printf("joints is not reshapeble to vectors of length %d (the number of joints)\n", num_joints);
                    request_valid_ = false;
                    return;
                }

                // reshape to 2d vector
                joints.reserve(joints_flat.size() / num_joints);
                for (size_t i = 0; i < joints_flat.size(); i += num_joints) {
                    joints.emplace_back(joints_flat.begin() + i,
                                                 joints_flat.begin() + i + num_joints);
                }
        }

        pvxs::shared_array<const double> joints_flat = {};
        std::vector<std::vector<double>> joints = {};

        static std::optional<FKRequest> build(pvxs::Value& value, unsigned int num_joints) {
            try {
                auto fk_req = FKRequest(value, num_joints);
                if (fk_req.request_valid_) {
                    return fk_req;
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

    struct FKResponse {
        bool valid = false;
        std::string description = "";
        pvxs::shared_array<double> poses_flat = {};

        pvxs::TypeDef response() {
            return pvxs::TypeDef(pvxs::TypeCode::Struct, {
                pvxs::members::Bool("valid"),
                pvxs::members::String("description"),
                pvxs::members::Float64A("poses"),
            });
        }

        pvxs::Value to_value() {
            auto value = response().create();
            value["valid"] = valid;
            value["description"] = description;
            value["poses"] = poses_flat.freeze();
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

    void fk_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value);
};
