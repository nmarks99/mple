#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <pvxs/iochooks.h>
#include <pvxs/nt.h>
#include <pvxs/server.h>
#include <pvxs/sharedpv.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/math.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "mple/adapter_registry.hpp"
#include "system_model.hpp"
#include "collisions.hpp"
#include "planner.hpp"
#include "rpc.hpp"
#include "scene.hpp"

namespace mple {

class Driver {
  public:
    Driver(const std::string& adapter_name, const std::string& prefix, const std::string& scene_objects_file);

  private:
    Adapter* adapter_;
    SystemModel system_;

    // Scene stuff
    std::mutex scene_mutex_;
    std::shared_ptr<const SceneDescription> scene_description_;
    std::shared_ptr<const CollisionChecker> collision_checker_;
    uint64_t scene_revision_ = 0;
    pvxs::Value scene_objects_value_;
    pvxs::server::SharedPV scene_objects_pv_ = pvxs::server::SharedPV::buildMailbox();
    pvxs::Value eviz_markers_value_;
    pvxs::server::SharedPV eviz_markers_pv_ = pvxs::server::SharedPV::buildReadonly();

    // Execution (motion) state
    // Could be expanded to show running for "long" running operations (e.g. planning)
    enum class ExecutionState {
        Idle,
        Running,
        StopRequested,
    };
    std::atomic<ExecutionState> execution_state_ = ExecutionState::Idle;

    // Status PV
    pvxs::Value status_value_;
    pvxs::server::SharedPV status_pv_ = pvxs::server::SharedPV::buildReadonly();

    // Stop PV
    pvxs::Value stop_value_;
    pvxs::server::SharedPV stop_pv_ = pvxs::server::SharedPV::buildMailbox();

    // PVs for all our provided services (via RPCs)
    pvxs::server::SharedPV plan_pv_ = pvxs::server::SharedPV::buildReadonly();
    pvxs::server::SharedPV validate_pv_ = pvxs::server::SharedPV::buildReadonly();
    pvxs::server::SharedPV execute_pv_ = pvxs::server::SharedPV::buildReadonly();
    pvxs::server::SharedPV ik_pv_ = pvxs::server::SharedPV::buildReadonly();
    pvxs::server::SharedPV fk_pv_ = pvxs::server::SharedPV::buildReadonly();

    struct SceneSnapshot {
        std::shared_ptr<const SceneDescription> scene;
        std::shared_ptr<const CollisionChecker> checker;
    };

    SceneSnapshot scene_snapshot();

    std::optional<std::string> check_state_collision(const SceneSnapshot& snapshot,
                                                     CollisionChecker::Workspace& ws,
                                                     const std::vector<double>& state);

    // Checks the waypoints and the straight joint-space segments between them, with states spaced
    // at most resolution apart
    std::optional<std::string> check_path_collision(const SceneSnapshot& snapshot,
                                                    CollisionChecker::Workspace& ws,
                                                    const std::vector<std::vector<double>>& waypoints,
                                                    double resolution);

    void scene_objects_put_callback(pvxs::server::SharedPV& pv, std::unique_ptr<pvxs::server::ExecOp>&& op,
                                    pvxs::Value&& put_value);

    void stop_rpc_callback(std::unique_ptr<pvxs::server::ExecOp>&& op, pvxs::Value&& put_value);

    void validate_rpc_callback(std::unique_ptr<pvxs::server::ExecOp>&& op, pvxs::Value&& request_value);

    void plan_rpc_callback(std::unique_ptr<pvxs::server::ExecOp>&& op, pvxs::Value&& request_value);

    void execute_rpc_callback(std::unique_ptr<pvxs::server::ExecOp>&& op, pvxs::Value&& request_value);

    void execute_trajectory(const ExecuteRequest& req);

    void execute_steps(const ExecuteRequest& req);

    void ik_rpc_callback(std::unique_ptr<pvxs::server::ExecOp>&& op, pvxs::Value&& request_value);

    void fk_rpc_callback(std::unique_ptr<pvxs::server::ExecOp>&& op, pvxs::Value&& request_value);
};

} // namespace mple
