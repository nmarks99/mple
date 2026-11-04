#include <thread>
#include <charconv>
#include <optional>
#include "mple_driver.hpp"
#include "planner.hpp"
#include "trajectory.hpp"

#include "iocsh.h"
#include "epicsExport.h"

// TODO: Remove: this was just for debugging
template <typename T>
std::optional<std::string> vector_to_string(const std::vector<T>& vec) {

    static_assert(std::is_arithmetic_v<T>);

    if (vec.empty()) {
        return "()";
    }

    // Guess how much space we'll need and reserve it
    std::string out("(");
    const size_t GUESS_NUMBER_OF_CHARS = (vec.size() * 8) + 2;
    out.reserve(GUESS_NUMBER_OF_CHARS);

    const size_t buf_size = 64;
    char buf[buf_size];

    // Process first to deal with trailing delimiter
    auto it = vec.begin();
    auto result = std::to_chars(buf, buf + buf_size, *it);
    if (result.ec != std::errc()) {
        return std::nullopt;
    }
    out.append(buf, result.ptr);
    ++it;

    // append a comma then the next converted number each time
    for (; it != vec.end(); ++it) {
        out.push_back(',');
        result = std::to_chars(buf, buf + buf_size, *it);
        if (result.ec != std::errc()) {
            return std::nullopt;
        }
        out.append(buf, result.ptr);
    }
    out.push_back(')');

    return out;
}

MpleDriver::MpleDriver(const std::string& adapter_name, const std::string& prefix, const std::string& obstacles_file) {

    // Find the adapter
    adapter_ = find_adapter(adapter_name);
    if (!adapter_) {
        throw std::runtime_error("Can't find adapter " + std::string(adapter_name));
    }

    // Get information about the system and build the model
    auto model_config = adapter_->config();
    try {
        pinocchio::urdf::buildModel(model_config.urdf_path, model_);
        if (!model_.existFrame(model_config.base_frame, pinocchio::BODY)) {
            throw std::runtime_error("Base frame not found: " + model_config.base_frame);
        }
        if (!model_.existFrame(model_config.ee_frame, pinocchio::BODY)) {
            throw std::runtime_error("End-effector frame not found: " + model_config.ee_frame);
        }
        state_dim_ = static_cast<size_t>(model_.nv);
        model_data_ = pinocchio::Data(model_);
        base_frame_id_ = model_.getFrameId(model_config.base_frame, pinocchio::BODY);
        ee_frame_id_ = model_.getFrameId(model_config.ee_frame, pinocchio::BODY);
    } catch (const std::exception& e) {
        std::cerr << "Failed to build system model from URDF: " << e.what() << std::endl;
        std::cerr << "MPLE aborting\n";
        return;
    }

    // Load obstacles file
    if (!obstacles_file.empty()) {
        printf("Loading obstacles from json file %s\n", obstacles_file.c_str());
        collision_world_.add_from_json(obstacles_file);
    }

    namespace M = pvxs::members;
    status_value_ = pvxs::TypeDef(pvxs::TypeCode::Struct, {
        M::Bool("running"),
        M::UInt32("iter"),
        M::UInt32("max")
    }).create();
    status_pv_.open(status_value_);
    pvxs::ioc::server().addPV(prefix + "Status", status_pv_);

    // Used to stop a running mple task
    stop_value_ = pvxs::nt::NTScalar{}.create();
    stop_pv_.onPut([this](pvxs::server::SharedPV& pv,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& value) {
        stop_callback(std::move(op), std::move(value));
    });
    stop_pv_.open(stop_value_);
    pvxs::ioc::server().addPV(prefix + "Stop", stop_pv_);

    // Validate
    validate_pv_.onRPC([this](pvxs::server::SharedPV& pv,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
        validate_callback(std::move(op), std::move(request_value));
    });
    pvxs::ioc::server().addPV(prefix + "Validate", validate_pv_);

    // Execute
    execute_pv_.onRPC([this](pvxs::server::SharedPV& pv,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
        execute_callback(std::move(op), std::move(request_value));
    });
    pvxs::ioc::server().addPV(prefix + "Execute", execute_pv_);

    // Plan
    plan_pv_.onRPC([this](pvxs::server::SharedPV& pv,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
        plan_callback(std::move(op), std::move(request_value));
    });
    pvxs::ioc::server().addPV(prefix + "Plan", plan_pv_);

    // Forward Kinematics
    fk_pv_.onRPC([this](pvxs::server::SharedPV& pv,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
        fk_callback(std::move(op), std::move(request_value));
    });
    pvxs::ioc::server().addPV(prefix + "ComputeFK", fk_pv_);

    // Inverse Kinematics
    ik_pv_.onRPC([this](pvxs::server::SharedPV& pv,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
        ik_callback(std::move(op), std::move(request_value));
    });
    pvxs::ioc::server().addPV(prefix + "ComputeIK", ik_pv_);
}

void MpleDriver::stop_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
    auto expected = ExecutionState::Running;
    if (execution_state_.compare_exchange_strong(expected, ExecutionState::StopRequested)) {
        adapter_->stop();
    }
    op->reply();
}

void MpleDriver::plan_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {

    PlanResponse response;
    auto request = PlanRequest::build(request_value, state_dim_);

    if (!request) {
        printf("Failed to build PlanRequest\n");
        response.valid = false;
        response.description = "Invalid request";
        op->reply(response.to_value());
        return;
    }

    std::vector<double> start(request->start.begin(), request->start.end());
    if (start.empty()) {
        std::cout << "No starting state provided, using current state as start\n";
        start = adapter_->get_state();
    }
    const std::vector<double> goal(request->goal.begin(), request->goal.end());
    PlanResult result = plan(start, goal, request->num_points);

    response.valid = result.valid;
    response.description = std::move(result.description);
    response.trajectory_flat = pvxs::shared_array<double>(
        result.trajectory_flat.begin(), result.trajectory_flat.end());
    op->reply(response.to_value());
}

void MpleDriver::fk_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
    FKResponse response;
    auto request = FKRequest::build(request_value, state_dim_);

    if (!request) {
        printf("Failed to build FKRequest\n");
        response.valid = false;
        response.description = "Invalid request";
        op->reply(response.to_value());
        return;
    }

    response.poses_flat.resize(request->joints.size() * 6);
    size_t ind = 0;
    for (auto& wp : request->joints) {
        auto pose = adapter_->fk(wp);
        if (!pose) {
            if (wp.size() != static_cast<size_t>(model_.nq)) {
                response.valid = false;
                response.description = "MPLE FK requires " + std::to_string(model_.nq)
                    + " configuration values, but received " + std::to_string(wp.size());
                op->reply(response.to_value());
                return;
            }

            const Eigen::Map<const Eigen::VectorXd> q(wp.data(), wp.size());
            pinocchio::SE3 ee_pose;
            {
                std::lock_guard<std::mutex> lock(model_mutex_);
                pinocchio::framesForwardKinematics(model_, model_data_, q);
                ee_pose = model_data_.oMf[base_frame_id_].inverse()
                    * model_data_.oMf[ee_frame_id_];
            }

            const Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(ee_pose.rotation());
            pose = std::vector<double>{
                ee_pose.translation().x(), ee_pose.translation().y(), ee_pose.translation().z(),
                rpy.x(), rpy.y(), rpy.z(),
            };
        }

        if (pose->size() != 6) {
            response.valid = false;
            response.description = "Forward kinematics must return six pose values";
            op->reply(response.to_value());
            return;
        }

        for (const double value : *pose) {
            response.poses_flat[ind] = value;
            ++ind;
        }
    }

    response.valid = true;
    response.description = "Okay";
    op->reply(response.to_value());
}

void MpleDriver::ik_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
    IKResponse response;
    auto request = IKRequest::build(request_value);

    if (!request) {
        response.valid = false;
        response.description = "Invalid request";
        op->reply(response.to_value());
        return;
    }

    response.joint_angles_flat.resize(request->cartesian_waypoints.size() * model_.nv);
    size_t ind = 0;
    for (auto& wp : request->cartesian_waypoints) {
        auto joints = adapter_->ik(wp);
        if (!joints) {
            response.valid = false;
            auto wp_string = vector_to_string(wp);
            response.description = "Inverse kinematics failed for waypoint " + vector_to_string(wp).value_or("(?)");
            op->reply(response.to_value());
            return;
        }
        for (const auto& j : *joints) {
            response.joint_angles_flat[ind] = j;
            ind += 1;
        }
    }

    response.valid = true;
    response.description = "Okay";
    op->reply(response.to_value());
}

void MpleDriver::execute_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
    ExecuteResponse response;
    auto request = ExecuteRequest::build(request_value, state_dim_);

    if (!request) {
        response.error_code = -1;
        response.description = "Invalid request";
        op->reply(response.to_value());
        return;
    }

    auto expected = ExecutionState::Idle;
    if (!execution_state_.compare_exchange_strong(expected, ExecutionState::Running)) {
        response.error_code = -1;
        response.description = "Execution already running";
        op->reply(response.to_value());
        return;
    }

    std::thread exec_thread;
    try {
        exec_thread = std::thread([req = std::move(request.value()), this]() {
            status_value_["running"] = true;
            status_value_["iter"] = 0;
            status_value_["max"] = req.waypoints.size();
            status_pv_.post(status_value_);

            try {

                // Build the time parametrized trajectory with TOPP-RA
                Trajectory traj = build_trajectory(
                    req.waypoints, req.speed, req.acceleration);

                if (execution_state_.load() == ExecutionState::StopRequested) {
                    throw std::runtime_error("Execution stopped");
                }

                // Command the move
                if (!adapter_->move(traj)) {
                    throw std::runtime_error("Error in adapter::move");
                }

                // Stop may have been requested while adapter::move was running.
                if (execution_state_.load() == ExecutionState::StopRequested) {
                    adapter_->stop();
                }

                // Wait for the move to complete
                auto start = std::chrono::steady_clock::now();
                while (execution_state_.load() == ExecutionState::Running && adapter_->moving()) {
                    auto now = std::chrono::steady_clock::now();
                    auto elap_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
                    if (elap_ms.count() >= req.timeout * 1000.0) {
                        std::cerr << "Execution timed out (" << req.timeout << " sec)\n";
                        adapter_->stop();
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
            } catch (const std::exception& e) {
                std::cerr << "Execution failed: " << e.what() << "\n";
            } catch (...) {
                std::cerr << "Execution failed with unknown error\n";
            }

            status_value_["running"] = false;
            status_value_["iter"] = 0;
            status_pv_.post(status_value_);
            execution_state_.store(ExecutionState::Idle);
        });
    } catch (const std::exception& e) {
        execution_state_.store(ExecutionState::Idle);
        response.error_code = -1;
        response.description = std::string("Failed to start execution thread: ") + e.what();
        op->reply(response.to_value());
        return;
    }
    exec_thread.detach();

    response.error_code = 0;
    response.description = "Execution started";
    op->reply(response.to_value());
}

void MpleDriver::validate_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {

    ValidateResponse response;
    auto request = ValidateRequest::build(request_value, state_dim_);

    if (!request) {
        response.valid = false;
        response.description = "Invalid request";
        op->reply(response.to_value());
        return;
    }

    // Collision, workspace, and joint limit checks require both joint
    // angles and TCP pose, hence why we need FK and IK depending on
    // which waypoints were given to the Validate method
    std::optional<std::string> hit;
    if (!request->joint_waypoints.empty()) {
        for (auto& wp : request->joint_waypoints) {
            auto tcp_pose = adapter_->fk(wp);
            if (!tcp_pose) {
                printf("Adapter does not provide FK. mple must do it.\n");
                // TODO: Compute FK here.
                // Probably need URDF or DH params
                break;
            } else {
                // TODO: check joint and workspace limits
                Position p{tcp_pose->at(0), tcp_pose->at(1), tcp_pose->at(2)};
                auto hit = collision_world_.check_point(p);
                if (hit) { break; }
            }
        }
    } else if (!request->cartesian_waypoints.empty()) {
        for (auto& wp : request->cartesian_waypoints) {
            auto joints = adapter_->ik(wp);
            if (!joints) {
                printf("Adapter does not provide IK (or it failed?). mple must do it.\n");
                // TODO: Compute IK here
                // Probably need URDF or DH params
                break;
            } else {
                // TODO: check joint and workspace limits
                Position p{wp[0], wp[1], wp[2]};
                auto hit = collision_world_.check_point(p);
                if (hit) { break; }
            }
        }
    }

    if (hit) {
        response.valid = false;
        response.description = "Collision with " + *hit;
    } else {
        response.valid = true;
        response.description = "No collisions detected";
    }

    op->reply(response.to_value());
}


// register function for iocsh
extern "C" void MpleDriverConfig(const char* adapter_name, const char* prefix, const char* obstacles_file) {
    new MpleDriver(adapter_name, prefix, obstacles_file);
}

static const iocshArg MpleDriverArg0 = {"Adapter name", iocshArgString};
static const iocshArg MpleDriverArg1 = {"IOC prefix", iocshArgString};
static const iocshArg MpleDriverArg2 = {"Obstacles json file", iocshArgString};
static const iocshArg* const MpleDriverArgs[3] = {&MpleDriverArg0, &MpleDriverArg1, &MpleDriverArg2};
static const iocshFuncDef MpleDriverFuncDef = {"MpleConfig", 3, MpleDriverArgs};

static void MpleDriverCallFunc(const iocshArgBuf* args) { MpleDriverConfig(args[0].sval, args[1].sval, args[2].sval); }

void MpleDriverRegister(void) { iocshRegister(&MpleDriverFuncDef, MpleDriverCallFunc); }

extern "C" {
epicsExportRegistrar(MpleDriverRegister);
}
