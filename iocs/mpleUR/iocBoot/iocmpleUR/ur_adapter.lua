local epics = require("epics")
local osi = require("osi")

-- TODO: make configurable by adding user macros
-- as global string variables to the lua state
P = "mpleUR:"

function get_num_joints()
    return 6
end

function get_joint_positions()
    return epics.get(P .. "Receive:Joints")
end

function get_tcp_pose()
    return epics.get(P .. "Receive:Pose")
end

function moving()
    local moving = epics.get(P .. "Control:Moving")
    if (moving == 1) then
        return true
    else
        return false
    end
end

function move(trajectory)
    if #trajectory == 0 or (#trajectory % get_num_joints()) ~= 0 then
        return false
    end

    local trajectory_deg = {}
    for i, value in ipairs(trajectory) do
        trajectory_deg[i] = math.deg(value)
    end

    epics.put(P .. "Control:ServoTraj", trajectory_deg)
    epics.put(P .. "Control:ServoStart", 1)

    -- HACK: wait for motion to start
    osi.sleep(0.2)

    return true
end

function stop()
    epics.put(P .. "Control:Stop.PROC", 1)
end
