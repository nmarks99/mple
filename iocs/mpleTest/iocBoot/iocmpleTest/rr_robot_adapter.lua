local epics = require("epics")
local osi = require("osi")

P = "mpleTest:"

-- Link lengths in m
L1 = 300 / 1000.0
L2 = 250 / 1000.0

function get_num_joints()
    return 2
end

function get_joint_positions()
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

-- TODO:
-- function is_moving()
    -- return epics.get(P .. "Moving");
-- end

function move(joints, speed, accel)
    -- Commands a joint move and blocks until complete or error
    print("Starting move...")
    -- print("J1Target = "..math.deg(joints[1]))
    -- print("J2Target = "..math.deg(joints[2]))
    -- print("Speed = "..math.deg(speed))
    -- print("Accel = "..math.deg(accel))
    epics.put(P.. "J1Target.VAL", math.deg(joints[1]))
    epics.put(P.. "J2Target.VAL", math.deg(joints[2]))
    epics.put(P .. "Move.PROC", 1)

    osi.sleep(0.1) -- HACK: wait for motion to start
    while true do
        if (epics.get(P .. "Moving") == 0) then
            break
        end
        osi.sleep(0.01)
    end
    print("Move done")
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
    local th2 = math.acos((x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2))
    local th1 = math.atan(y/x) + math.atan((L2*math.sin(th2)) / (L1+L2*math.cos(th2)))
    return {th1, th2}
end

function stop()
    epics.put(P .. "Stop.PROC", 1)
end
