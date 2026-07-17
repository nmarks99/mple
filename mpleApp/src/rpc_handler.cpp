#include <thread>
#include "mple_driver.hpp"
#include "rpc_handler.hpp"

RPCHandler::RPCHandler(MpleDriver* driver, MpleAdapter* adapter, const std::string& prefix) : driver_(driver), adapter_(adapter) {
    assert(driver_ != nullptr && "driver must not be null");
    assert(adapter_ != nullptr && "adapter must not be null");

    // Set up PVXS RPC callbacks for our custom PVs
    //
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

void RPCHandler::ik_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
    IKResponse response;
    auto request = IKRequest::build(request_value);

    if (!request) {
        response.valid = false;
        response.description = "Invalid request";
        op->reply(response.to_value());
        return;
    }

    // FIX: use adapter_->get_num_joints instead of hardcoded *2
    response.joint_angles_flat.resize(request->cartesian_waypoints.size() * 2);
    size_t ind = 0;
    for (auto& wp : request->cartesian_waypoints) {
        printf("wp = %lf,%lf,%lf,%lf,%lf,%lf\n", wp[0],wp[1],wp[2],wp[3],wp[4],wp[5]);
        auto joints = adapter_->ik(wp);
        if (!joints) {
            response.valid = false;
            response.description = "Inverse kinematics failed";
            op->reply(response.to_value());
            return;
        }
        for (const auto& j : *joints) {
            response.joint_angles_flat[ind] = j;
            ind += 1;
        }
    }

    for (const auto& v : response.joint_angles_flat) {
        printf("%lf\n", v);
    }

    response.valid = true;
    response.description = "Okay";
    op->reply(response.to_value());
}

void RPCHandler::execute_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
    ExecuteResponse response;
    auto request = ExecuteRequest::build(request_value, adapter_);

    if (!request) {
        response.error_code = -1;
        response.description = "Invalid request";
        op->reply(response.to_value());
        return;
    }

    std::thread exec_thread([req = std::move(request.value()), this]() {
        driver_->lock();
        driver_->setIntegerParam(driver_->idx_mt_active_, 1);
        driver_->callParamCallbacks();
        driver_->unlock();

        adapter_->execute_trajectory(req.joint_waypoints, req.speed, req.acceleration);

        driver_->lock();
        driver_->setIntegerParam(driver_->idx_mt_active_, 0);
        driver_->callParamCallbacks();
        driver_->unlock();
    });
    exec_thread.detach();

    // Reply right away saying we started the process
    response.error_code = 0;
    response.description = "Execution started";
    op->reply(response.to_value());
}

void RPCHandler::validate_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
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
                auto hit = driver_->collision_world.check_point(p);
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
                auto hit = driver_->collision_world.check_point(p);
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
