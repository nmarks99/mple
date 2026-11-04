local epics = require("epics")
local osi = require("osi")

function config()
    return {
        urdf = MPLE .. "/description/ur5e.urdf",
        base_frame = "base",
        ee_frame = "tool0"
    }
end

function get_state()
   local joints = epics.get(P .. "Receive:Joints")
   for i, joint in ipairs(joints) do
       joints[i] = math.rad(joint)
   end
   return joints
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

    -- trajectory argument is of the form:
    -- {
        -- {
            -- time = 0,
            -- position = {...},
            -- velocity = {...},
            -- acceleration = {...},
        -- } ...,
    -- }

    -- Convert the trajectory to the format we
    -- need to provide to the urRobot ServoTraj PV
    local traj_flat = {}
    for _, point in ipairs(trajectory) do
        table.insert(traj_flat, point.time);
        for _, pos in ipairs(point.position) do
            table.insert(traj_flat, pos);
        end
        for _, vel in ipairs(point.velocity) do
            table.insert(traj_flat, vel);
        end
        for _, acc in ipairs(point.acceleration) do
            table.insert(traj_flat, acc);
        end
    end
    if #traj_flat < 38 or (#traj_flat % 19) ~= 0 then
        print("Failed to convert trajectory to proper format")
        return false
    end

    epics.put(P .. "Control:ServoTraj", traj_flat)
    epics.put(P .. "Control:ServoStart", 1)

    -- HACK: wait for motion to start
    osi.sleep(0.2)
    return true
end

function stop()
    epics.put(P .. "Control:ServoStop", 1)
end
