#include <thread>
#include <charconv>
#include <optional>
#include "mple_driver.hpp"
// #include "trajectory.hpp"

#include "iocsh.h"
#include "epicsExport.h"

#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

// #include <toppra/algorithm/toppra.hpp>
// #include <toppra/constraint/linear_joint_acceleration.hpp>
// #include <toppra/constraint/linear_joint_velocity.hpp>
// #include <toppra/geometric_path/piecewise_poly_path.hpp>
// #include <toppra/parametrizer/spline.hpp>
// #include <toppra/solver/seidel.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

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
    printf("Stop callback called!\n");
    // Stops the current motion but we must also stop the execute
    // callback thread from commanding further motion
    adapter_->stop();

    // TODO: Stop the execute callback thread from commanding further motion
    // - Set flag should_stop_ = true, where should_stop_ is atomic<bool>
    should_stop_ = true;
    op->reply();
}
void MpleDriver::plan_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {

    PlanResponse response;
    const auto njoints = adapter_->get_num_joints();
    auto request = PlanRequest::build(request_value, adapter_->get_num_joints());

    if (!request) {
        printf("Failed to build PlanRequest\n");
        response.valid = false;
        response.description = "Invalid request";
        op->reply(response.to_value());
        return;
    }

    // Create the configuration space
    auto space = std::make_shared<ob::CompoundStateSpace>();
    space->addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0);
    space->addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0);

    // Set up geometric planner with SimpleSetup and RRTConnect planner
    og::SimpleSetup setup(space);
    setup.setStateValidityChecker([](const ob::State *) { return true; });
    setup.setPlanner(std::make_shared<og::RRTConnect>(setup.getSpaceInformation()));

    // Set start and goal states
    std::vector<double> req_start(request->start.begin(), request->start.end());
    if (req_start.size() == 0) {
        std::cout << "No starting state provided, using current state as start\n";
        req_start = adapter_->get_joint_positions();
    }
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    auto *start_state = start->as<ob::CompoundStateSpace::StateType>();
    auto *goal_state = goal->as<ob::CompoundStateSpace::StateType>();
    start_state->as<ob::SO2StateSpace::StateType>(0)->value = req_start[0];
    start_state->as<ob::SO2StateSpace::StateType>(1)->value = req_start[1];
    goal_state->as<ob::SO2StateSpace::StateType>(0)->value = request->goal[0];
    goal_state->as<ob::SO2StateSpace::StateType>(1)->value = request->goal[1];
    space->enforceBounds(goal.get());
    setup.setStartAndGoalStates(start, goal, 1e-9);

    // Solve
    const ob::PlannerStatus status = setup.solve(1.0);
    if (status != ob::PlannerStatus::EXACT_SOLUTION) {
        response.valid = false;
        response.description = std::string("Failed to find an exact path: " + status.asString());
        op->reply(response.to_value());
        return;
    }

    setup.simplifySolution();
    og::PathGeometric &path = setup.getSolutionPath();
    if (!path.check()) {
        response.valid = false;
        response.description = std::string("OMPL returned an invalid path");
        op->reply(response.to_value());
        return;
    }

    const unsigned int total_points = request->num_points + 2;
    response.trajectory_flat.resize(total_points * njoints);
    path.interpolate(total_points);

    constexpr double two_pi = 6.28318530717958647692;
    double previousQ1 = 0.0;
    double previousQ2 = 0.0;
    std::cout << std::fixed << std::setprecision(6);
    for (std::size_t i = 0; i < path.getStateCount(); ++i)
    {
        const auto *state = path.getState(i)->as<ob::CompoundStateSpace::StateType>();
        const double wrappedQ1 = state->as<ob::SO2StateSpace::StateType>(0)->value;
        const double wrappedQ2 = state->as<ob::SO2StateSpace::StateType>(1)->value;
        const double q1 = previousQ1 + std::remainder(wrappedQ1 - previousQ1, two_pi);
        const double q2 = previousQ2 + std::remainder(wrappedQ2 - previousQ2, two_pi);
        response.trajectory_flat.at(i * njoints) = q1;
        response.trajectory_flat.at(i * njoints + 1) = q2;
        previousQ1 = q1;
        previousQ2 = q2;
    }

    response.valid = true;
    response.description = "Success";
    op->reply(response.to_value());
}

void MpleDriver::fk_callback(std::unique_ptr<pvxs::server::ExecOp>&& op,
                          pvxs::Value&& request_value) {
    FKResponse response;
    auto request = FKRequest::build(request_value, adapter_->get_num_joints());

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
        auto joints = adapter_->fk(wp);
        if (!joints) {
            response.valid = false;
            auto wp_string = vector_to_string(wp);
            response.description = "Forward kinematics failed for joint angles " + vector_to_string(wp).value_or("(?)");
            op->reply(response.to_value());
            return;
        }
        for (const auto& j : *joints) {
            response.poses_flat[ind] = j;
            ind += 1;
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
        // constexpr double timestep = 0.002;

        status_value_["running"] = true;
        status_value_["iter"] = 0;
        status_value_["max"] = req.joint_waypoints.size();
        status_pv_.post(status_value_);

        //////////////////////////////////////////////////////////////////////////////////////
        // TODO: Replace this with TOPP-RA, which should be able to give us trajectory_flat
        // Need adapter_->get_robot_info or similar which tell us about joint limits,
        // controller servo loop timestep, and other info we might need

        const int dof = req.num_joints;
        constexpr double sample_period = 0.02;
        constexpr double max_velocity = 0.2;
        constexpr double max_acceleration = 1.0;

        // Store waypoints from user in toppra::Vectors
        toppra::Vectors waypoints;
        for (auto& wp : req.joint_waypoints) {
            waypoints.push_back(Eigen::Map<const toppra::Vector>(wp.data(), wp.size()));
        }

        // Assign parameter values s to the geometric waypoints
        // TODO: This would be better: s[i] = s[i - 1] + norm(q[i] - q[i - 1])
        toppra::Vector s_vec = toppra::Vector::LinSpaced(waypoints.size(), 0.0, waypoints.size() - 1);

        // Create the cubic spline describing joint positions along the geometric path
        toppra::BoundaryCond natural{2, toppra::Vector::Zero(dof)};
        auto path = std::make_shared<toppra::PiecewisePolyPath>(
            toppra::PiecewisePolyPath::CubicSpline(
                waypoints, s_vec, toppra::BoundaryCondFull{natural, natural}));

        // Set contraints for the velocity and acceleration
        const toppra::Vector velocity_limit = toppra::Vector::Constant(dof, max_velocity);
        const toppra::Vector acceleration_limit = toppra::Vector::Constant(dof, max_acceleration);
        toppra::LinearConstraintPtrs constraints{
            std::make_shared<toppra::constraint::LinearJointVelocity>(
                -velocity_limit, velocity_limit),
            std::make_shared<toppra::constraint::LinearJointAcceleration>(
                -acceleration_limit, acceleration_limit),
        };

        // Compute a time-optimal path scaling s(t) subject to the joint limits
        toppra::algorithm::TOPPRA algorithm{constraints, path};
        algorithm.solver(std::make_shared<toppra::solver::Seidel>());
        const auto result = algorithm.computePathParametrization(0.0, 0.0);
        if (result != toppra::ReturnCode::OK) {
            std::cerr << "TOPP-RA could not parameterize the path: "
                      << algorithm.getErrorMessage() << '\n';
            return;
        }

        // Construct the time-domain trajectory q(t) from the parameterization.
        const auto &data = algorithm.getParameterizationData();
        toppra::parametrizer::Spline trajectory{path, data.gridpoints, data.parametrization};
        const double duration = trajectory.pathInterval()[1];
        const int regular_points = static_cast<int>(std::floor(duration / sample_period));
        const bool endpoint_is_regular = std::abs(regular_points * sample_period - duration) < 1e-12;
        const int point_count = regular_points + (endpoint_is_regular ? 1 : 2);

        // Store the result in a flat vector: (time, positions[6], velocities[6], accelerations[6])
        std::vector<double> trajectory_flat;
        for (int index = 0; index < point_count; ++index) {
            const double time = index <= regular_points ? index * sample_period : duration;
            trajectory_flat.push_back(time);
            for (size_t i = 0; i < 3; ++i) {
                auto vec = trajectory.eval_single(time, i);
                for (auto& v : vec) {
                    trajectory_flat.push_back(v);
                }
            }
        }
        //////////////////////////////////////////////////////////////////////////////////////

        if (adapter_->move(trajectory_flat)) {
            while (adapter_->moving()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        } else {
            printf("adapter_->move returned error. Aborting motion\n");
        }

        status_value_["running"] = false;
        status_value_["iter"] = 0;
        status_pv_.post(status_value_);
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
