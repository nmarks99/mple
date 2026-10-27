local epics = require("epics")
local osi = require("osi")

-- TODO: make configurable by adding user macros
-- as global string variables to the lua state
P = "mpleTest:"

-- Link lengths in m
L1 = 300 / 1000.0
L2 = 250 / 1000.0

function num_axes()
    return 2
end

function get_axis_positions()
    -- Get joint angles, convert from deg->rad
    local j1 = math.rad(epics.get(P .. "J1.VAL"))
    local j2 = math.rad(epics.get(P .. "J2.VAL"))
    return {j1, j2}
end

function get_tcp_pose()
    -- Get TCP pose, convert from mm->m
    local x = epics.get(P .. "X.VAL") / 1000.0
    local y = epics.get(P .. "Y.VAL") / 1000.0
    return {x, y};
end

function moving()
    local rval = epics.get(P .. "Moving")
    if (rval == 1) then
        return true
    else
        return false
    end
end


function move(joints, speed, accel)
    -- Commands a joint move and blocks until complete or error
    epics.put(P.. "J1Target.VAL", math.deg(joints[1]))
    epics.put(P.. "J2Target.VAL", math.deg(joints[2]))
    epics.put(P .. "Move.PROC", 1)

    print("Moving to WP: " .. math.deg(joints[1]) .. "," .. math.deg(joints[2]))

    -- HACK: wait a bit until Moving is 1
    osi.sleep(0.1)

    return true
end

-- returns TCP pose given joint angles
function fk(joints)
    return {
        L1 * math.cos(joints[1]) + L2*math.cos(joints[1]+joints[2]),
        L1 * math.sin(joints[1]) + L2*math.sin(joints[1]+joints[2]),
        0.0
    }
end

-- returns joint angles needed to achieve given TCP pose
function ik(tcp_pose)
    local x = tcp_pose[1]
    local y = tcp_pose[2]

    -- Workspace boundary check with small epsilon for numerical jitter
    local d_sq = x*x + y*y
    local max_reach = L1 + L2
    local min_reach = math.abs(L1 - L2)
    local eps = 1e-6
    local max_limit = max_reach + eps
    local min_limit = min_reach - eps
    if d_sq > (max_limit * max_limit) or d_sq < (min_limit * min_limit) then
        -- unreachable
        return nil
    end

    -- need to clamp this due to numerical rounding errors
    local op = (d_sq - L1*L1 - L2*L2) / (2*L1*L2)
    op = math.max(-1.0, math.min(1.0, op))

    local th2 = math.acos(op)
    local th1 = math.atan(y, x) - math.atan((L2 * math.sin(th2)), (L1 + L2 * math.cos(th2)))

    return {th1, th2}
end


function stop()
    epics.put(P .. "Stop.PROC", 1)
end
