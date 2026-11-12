#include <thread>
#include <charconv>
#include <optional>
#include "mple_driver.hpp"
#include "planner.hpp"
#include "trajectory.hpp"
#include "kinematics.hpp"

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

MpleDriver::MpleDriver(const std::string& adapter_name, const std::string& prefix, const std::string& scene_objects_file) {

    // Find the adapter
    adapter_ = find_adapter(adapter_name);
    if (!adapter_) {
        throw std::runtime_error("Can't find adapter " + std::string(adapter_name));
    }

    // Get information about the system and build the model
    auto model_config = adapter_->config();
    try {
        pinocchio::urdf::buildModel(model_config.urdf_path, system_.model);
        if (!system_.model.existFrame(model_config.base_frame, pinocchio::BODY)) {
            throw std::runtime_error("Base frame not found: " + model_config.base_frame);
        }
        if (!system_.model.existFrame(model_config.ee_frame, pinocchio::BODY)) {
            throw std::runtime_error("End-effector frame not found: " + model_config.ee_frame);
        }
        if (system_.model.nq != system_.model.nv) {
            throw std::runtime_error("MPLE requires Pinocchio models with nq == nv");
        }
        system_.state_dim = static_cast<size_t>(system_.model.nv);
        system_.data = pinocchio::Data(system_.model);
        system_.base_frame_id = system_.model.getFrameId(model_config.base_frame, pinocchio::BODY);
        system_.ee_frame_id = system_.model.getFrameId(model_config.ee_frame, pinocchio::BODY);
    } catch (const std::exception& e) {
        std::cerr << "Failed to build system model from URDF: " << e.what() << std::endl;
        std::cerr << "MPLE aborting\n";
        return;
    }

    // Load scene objects file
    auto initial_scene = std::make_shared<SceneDescription>();
    if (!scene_objects_file.empty()) {
        initial_scene->add_from_json(scene_objects_file);
        printf("Loaded scene objects from json file %s\n", scene_objects_file.c_str());
    }
    scene_description_ = initial_scene;

    // Set initial value of scene objects PV from planning scene
    scene_objects_value_ = scene_description_->to_pvxs_value(scene_revision_);
    scene_objects_pv_.onPut([this](pvxs::server::SharedPV& pv,
                                   std::unique_ptr<pvxs::server::ExecOp>&& op,
                                   pvxs::Value&& value) {
        scene_objects_put_callback(pv, std::move(op), std::move(value));
    });
    scene_objects_pv_.open(scene_objects_value_);
    pvxs::ioc::server().addPV(prefix + "SceneObjects", scene_objects_pv_);

    // Create status PV
    // TODO: is this properly updated/should it have any more fields?
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
        stop_rpc_callback(std::move(op), std::move(value));
    });
    stop_pv_.open(stop_value_);
    pvxs::ioc::server().addPV(prefix + "Stop", stop_pv_);

    // Validate
    validate_pv_.onRPC([this](pvxs::server::SharedPV& pv,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
        validate_rpc_callback(std::move(op), std::move(request_value));
    });
    pvxs::ioc::server().addPV(prefix + "Validate", validate_pv_);

    // Plan
    plan_pv_.onRPC([this](pvxs::server::SharedPV& pv,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
        plan_rpc_callback(std::move(op), std::move(request_value));
    });
    pvxs::ioc::server().addPV(prefix + "Plan", plan_pv_);

    // Execute
    execute_pv_.onRPC([this](pvxs::server::SharedPV& pv,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
        execute_rpc_callback(std::move(op), std::move(request_value));
    });
    pvxs::ioc::server().addPV(prefix + "Execute", execute_pv_);

    // Forward Kinematics
    fk_pv_.onRPC([this](pvxs::server::SharedPV& pv,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
        fk_rpc_callback(std::move(op), std::move(request_value));
    });
    pvxs::ioc::server().addPV(prefix + "ComputeFK", fk_pv_);

    // Inverse Kinematics
    ik_pv_.onRPC([this](pvxs::server::SharedPV& pv,
                          std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
        ik_rpc_callback(std::move(op), std::move(request_value));
    });
    pvxs::ioc::server().addPV(prefix + "ComputeIK", ik_pv_);
}

std::shared_ptr<const SceneDescription> MpleDriver::scene_snapshot() {
    std::lock_guard<std::mutex> lock(scene_mutex_);
    return scene_description_;
}

std::optional<std::string> MpleDriver::check_state_collision(
    const SceneDescription& scene, const std::vector<double>& state) {

    auto pose = adapter_->fk(state);
    if (!pose) {
        pose = forward_kinematics(system_, state);
    }

    const Position position{pose->at(0), pose->at(1), pose->at(2)};
    return scene.check_point(position);
}

void MpleDriver::scene_objects_put_callback(pvxs::server::SharedPV& pv,
                                            std::unique_ptr<pvxs::server::ExecOp>&& op,
                                            pvxs::Value&& put_value) {
    std::shared_ptr<const SceneDescription> new_scene;
    uint64_t submitted_revision = 0;
    try {
        new_scene = std::make_shared<const SceneDescription>(SceneDescription::from_pvxs_value(put_value));
        submitted_revision = put_value["revision"].as<uint64_t>();
    } catch (const std::exception& e) {
        op->error(std::string("Invalid scene: ") + e.what());
        return;
    }

    std::lock_guard<std::mutex> lock(scene_mutex_);

    if (submitted_revision != scene_revision_) {
        op->error("Scene revision conflict: current revision is " + std::to_string(scene_revision_) +
                  ", got " + std::to_string(submitted_revision));
        return;
    }

    auto new_value = new_scene->to_pvxs_value(scene_revision_ + 1);
    scene_description_ = std::move(new_scene);
    ++scene_revision_;
    scene_objects_value_ = new_value;
    pv.post(scene_objects_value_);
    op->reply();
}

void MpleDriver::stop_rpc_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
    auto expected = ExecutionState::Running;
    if (execution_state_.compare_exchange_strong(expected, ExecutionState::StopRequested)) {
        adapter_->stop();
    }
    op->reply();
}

void MpleDriver::plan_rpc_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {

    PlanResponse response;
    auto request = PlanRequest::build(request_value, system_.state_dim);

    if (!request) {
        printf("Failed to build PlanRequest\n");
        response.valid = false;
        response.description = "Invalid request";
        op->reply(response.to_value());
        return;
    }

    auto scene = scene_snapshot();

    std::vector<double> start(request->start.begin(), request->start.end());
    if (start.empty()) {
        std::cout << "No starting state provided, using current state as start\n";
        start = adapter_->get_state();
    }
    const std::vector<double> goal(request->goal.begin(), request->goal.end());
    PlanResult result = plan(
        start, goal,
        [this, &scene](const std::vector<double>& state) {
            return !check_state_collision(*scene, state);
        });

    response.valid = result.valid;
    response.description = std::move(result.description);
    response.trajectory_flat = pvxs::shared_array<double>(
        result.trajectory_flat.begin(), result.trajectory_flat.end());
    op->reply(response.to_value());
}

void MpleDriver::fk_rpc_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
    FKResponse response;
    auto request = FKRequest::build(request_value, system_.state_dim);

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
            if (wp.size() != static_cast<size_t>(system_.model.nq)) {
                response.valid = false;
                response.description = "MPLE FK requires " + std::to_string(system_.model.nq)
                    + " configuration values, but received " + std::to_string(wp.size());
                op->reply(response.to_value());
                return;
            }
            pose = forward_kinematics(system_, wp);
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

void MpleDriver::ik_rpc_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
    IKResponse response;
    auto request = IKRequest::build(request_value);

    if (!request) {
        response.valid = false;
        response.description = "Invalid request";
        op->reply(response.to_value());
        return;
    }

    response.joint_angles_flat.resize(request->cartesian_waypoints.size() * system_.model.nv);
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

void MpleDriver::execute_rpc_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
    ExecuteResponse response;
    auto request = ExecuteRequest::build(request_value, system_.state_dim);

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

void MpleDriver::validate_rpc_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {

    ValidateResponse response;
    auto request = ValidateRequest::build(request_value, system_.state_dim);

    if (!request) {
        response.valid = false;
        response.description = "Invalid request";
        op->reply(response.to_value());
        return;
    }

    auto scene = scene_snapshot();

    // Collision, workspace, and joint limit checks require both joint
    // angles and TCP pose, hence why we need FK and IK depending on
    // which waypoints were given to the Validate method
    std::optional<std::string> hit;
    if (!request->joint_waypoints.empty()) {
        for (const auto& wp : request->joint_waypoints) {
            hit = check_state_collision(*scene, wp);
            if (hit) {
                break;
            }
        }
    } else if (!request->cartesian_waypoints.empty()) {
        for (auto& wp : request->cartesian_waypoints) {
            auto joints = adapter_->ik(wp);
            if (!joints) {
                std::cerr << "Adapter does not provide IK (or it failed?). mple must do it.\n";
                // TODO: Compute IK here
                // Probably need URDF or DH params
                break;
            } else {
                // TODO: check joint and workspace limits
                Position p{wp[0], wp[1], wp[2]};
                hit = scene->check_point(p);
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
extern "C" void MpleDriverConfig(const char* adapter_name, const char* prefix, const char* scene_objects_file) {
    new MpleDriver(adapter_name, prefix, scene_objects_file);
}

static const iocshArg MpleDriverArg0 = {"Adapter name", iocshArgString};
static const iocshArg MpleDriverArg1 = {"IOC prefix", iocshArgString};
static const iocshArg MpleDriverArg2 = {"Scene json file", iocshArgString};
static const iocshArg* const MpleDriverArgs[3] = {&MpleDriverArg0, &MpleDriverArg1, &MpleDriverArg2};
static const iocshFuncDef MpleDriverFuncDef = {"MpleConfig", 3, MpleDriverArgs};

static void MpleDriverCallFunc(const iocshArgBuf* args) { MpleDriverConfig(args[0].sval, args[1].sval, args[2].sval); }

void MpleDriverRegister(void) { iocshRegister(&MpleDriverFuncDef, MpleDriverCallFunc); }

extern "C" {
epicsExportRegistrar(MpleDriverRegister);
}
