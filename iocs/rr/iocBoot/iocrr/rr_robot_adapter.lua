local epics = require("epics")

-- Hack to make LSP happy to avoid "undefined global"
P = P
MPLE = MPLE

NUM_JOINTS = 2

-- Link lengths in m
L1 = 300 / 1000.0
L2 = 250 / 1000.0

function config()
    return {
        description_dir = MPLE .. "/description",
        urdf = "rr_robot.urdf",
        base_frame = "base_link",
        ee_frame = "tool0"
    }
end

function get_state()
    -- Get joint angles, convert from deg->rad
    local j1 = math.rad(epics.get(P .. "J1.VAL"))
    local j2 = math.rad(epics.get(P .. "J2.VAL"))
    return {j1, j2}
end

function moving()
    local rval = epics.get(P .. "Moving")
    if (rval == 1) then
        return true
    else
        return false
    end
end

function move_step(target)
    -- target argument is of the form:
    -- {
    --     time = segment duration,
    --     position = {...},
    --     velocity = {...}, peak speed of each joint
    --     acceleration = {...}, acceleration of each joint
    -- }
    -- All joints share the same ramp time (ACCL = velocity / acceleration),
    -- so scaling VELO keeps them synchronized along a straight joint space line.
    for i = 1, NUM_JOINTS do
        local vel = target.velocity[i]
        local acc = target.acceleration[i]
        if vel > 0 and acc > 0 then
            epics.put(P .. "m" .. i .. ".VELO", math.deg(vel))
            epics.put(P .. "m" .. i .. ".ACCL", vel / acc)
        end
    end

    epics.put(P .. "J1Target.VAL", math.deg(target.position[1]))
    epics.put(P .. "J2Target.VAL", math.deg(target.position[2]))
    epics.put(P .. "Move.PROC", 1)

    return true
end

-- returns joint angles needed to achieve given TCP pose (x,y,z,rx,ry,rz)
-- only x and y are used
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
