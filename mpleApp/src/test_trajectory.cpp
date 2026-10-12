#include <epicsGuard.h>
#include <epicsThread.h>
#include <epicsUnitTest.h>
#include <testMain.h>

#include <trajectory.hpp>

MAIN(testTrajectory) {
    testPlan(2);

    std::vector<double> q0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> q1 = {0.0, 1.57, 0.7, 0.7, 0.0, 0.0};
    TrajectoryParams params = {
        .max_speed = 5.0,
        .max_accel = 10.0,
        .dt = 0.002
    };
    auto traj = joint_trajectory(q0, q1, params);
    testDiag("Size = (%ld, %ld)\n", traj.size(), traj[0].size());
    testOk1(traj.size() == 478);
    testOk1(traj[0].size() == 6);
    return testDone();
}
