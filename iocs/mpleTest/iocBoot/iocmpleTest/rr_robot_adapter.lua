local epics = require("epics")

function get_num_joints()
    return 2
end

function get_joint_positions()
    local j1 = epics.get(P .. "J1.VAL")
    local j2 = epics.get(P .. "J2.VAL")
    return {j1, j2}
end

function get_tcp_pose()
    local x = epics.get(P .. "X.VAL")
    local y = epics.get(P .. "Y.VAL")
    return {x, y};
end

function stop()
    epics.put(P .. "Stop.PROC", 1)
end
