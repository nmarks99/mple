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
    local rval = epics.get(P .. "Control:Moving")
    if (rval == 1) then
        return true
    else
        return false
    end
end


function move(joints, speed, accel)
    epics.put(P.. "Control:J1Cmd.VAL", math.deg(joints[1]))
    epics.put(P.. "Control:J2Cmd.VAL", math.deg(joints[2]))
    epics.put(P.. "Control:J3Cmd.VAL", math.deg(joints[3]))
    epics.put(P.. "Control:J4Cmd.VAL", math.deg(joints[4]))
    epics.put(P.. "Control:J5Cmd.VAL", math.deg(joints[5]))
    epics.put(P.. "Control:J6Cmd.VAL", math.deg(joints[6]))
    epics.put(P .. "Control:moveJ.PROC", 1)

    -- HACK: wait a bit until Moving is 1
    osi.sleep(0.1)

    return true
end

function stop()
    epics.put(P .. "Control:Stop.PROC", 1)
end
