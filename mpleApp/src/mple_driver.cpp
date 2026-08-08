#include <thread>
#include <charconv>
#include <optional>
#include "mple_driver.hpp"

#include "iocsh.h"
#include "epicsExport.h"

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
    adapter_ = find_adapter(adapter_name);
    if (!adapter_) {
        throw std::runtime_error("Can't find adapter " + std::string(adapter_name));
    }

    if (!obstacles_file.empty()) {
        printf("Loading obstacles from json file %s\n", obstacles_file.c_str());
        collision_world_.add_from_json(obstacles_file);
    }


    // TODO: remove separate rnning when confirmed this works
    namespace M = pvxs::members;
    status_value_ = pvxs::TypeDef(pvxs::TypeCode::Struct, {
        M::Bool("running"),
        M::Int32("iter"),
        M::Int32("max")
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

    // Bool to signal if mple is running a motion task
    running_value_ = pvxs::nt::NTScalar{pvxs::TypeCode::Bool}.create();
    running_value_["value"] = false;
    running_pv_.open(running_value_);
    pvxs::ioc::server().addPV(prefix + "Running", running_pv_);

    // TODO: pv.open a value on these so clients can see them
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
    adapter_->stop();
    op->reply();
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

    response.joint_angles_flat.resize(request->cartesian_waypoints.size() * adapter_->get_num_joints());
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
    auto request = ExecuteRequest::build(request_value, adapter_);

    if (!request) {
        response.error_code = -1;
        response.description = "Invalid request";
        op->reply(response.to_value());
        return;
    }

    printf("Running execution thread\n");
    std::thread exec_thread([req = std::move(request.value()), this]() {
        running_value_["value"] = true;
        running_pv_.post(running_value_);

        bool ok = false;
        unsigned int count = 0;

        status_value_["running"] = true;
        status_value_["iter"] = 0;
        status_value_["max"] = req.joint_waypoints.size();

        for (auto& wp : req.joint_waypoints) {
            // wait for current motion to finish
            while (adapter_->moving()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }

            ok = adapter_->move(wp, req.speed, req.acceleration);
            if (!ok) {
                printf("adapter_->move returned error. Aborting motion\n");
                break;
            }
            count++;
            status_value_["iter"] = count;
            status_pv_.post(status_value_);
        }

        status_value_["running"] = false;
        status_value_["iter"] = 0;
        status_pv_.post(status_value_);

        running_value_["value"] = false;
        running_pv_.post(running_value_);
    });
    exec_thread.detach();

    // Reply right away saying we started the process
    response.error_code = 0;
    response.description = "Execution started";
    op->reply(response.to_value());
}

void MpleDriver::validate_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {

    ValidateResponse response;
    auto request = ValidateRequest::build(request_value, adapter_);

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
