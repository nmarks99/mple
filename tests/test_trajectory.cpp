#include <epicsGuard.h>
#include <epicsThread.h>
#include <epicsUnitTest.h>
#include <testMain.h>
#include <iostream>

#include <trajectory.hpp>

MAIN(testTrajectory) {
    testPlan(0);

    std::vector<std::vector<double>> waypoints = {
        {0, 0, 0, 0, 0, 0},
        {1, 1, 1, 1, 1, 1},
    };

    Trajectory traj = build_trajectory(waypoints, 1.0, 10.0);

    std::cout << traj.size() << std::endl;

    for (const auto& p : traj.back().position) {
        testOk1(p == 1.0);
    }

    return testDone();
}
