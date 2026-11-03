#include <epicsGuard.h>
#include <epicsThread.h>
#include <epicsUnitTest.h>
#include <testMain.h>
#include <cmath>

#include <trajectory.hpp>

MAIN(testTrajectory) {
    testPlan(0);

    testOk1(1==0);

    return testDone();
}
