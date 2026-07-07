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

    response.valid = true;
    response.description = "Success";
    printf("joint_waypoints.size() = %ld\n", request->joint_waypoints.size());
    // printf("cartesian_waypoints.size() = %ld\n", request->cartesian_waypoints.size());
    // printf("Got Resolution = %lf\n", request->resolution);
    op->reply(response.to_value());
}
