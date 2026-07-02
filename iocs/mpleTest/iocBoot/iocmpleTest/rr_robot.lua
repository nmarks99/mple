-- -----------------------------------------------
-- Implements a simulated planar 2 joint robot arm
-- -----------------------------------------------
local db = require("db")
local iocsh = require("iocsh")

-- Hack to make LSP happy to avoid "undefined global"
P = P
STATE = STATE

luaRegisterState(STATE)

-- Robot had 2 revolute joints
NUM_JOINTS = 2
-- Link lengths in mm
L1 = 300
L2 = 250

-- Create simulated motors for the two robot joints
INSTANCE = "motorSim"
CARD = 0
NUM_AXES = NUM_JOINTS
HIGH_LIM = 100000000
LOW_LIM = -100000000
HOME_POS = 0
iocsh.motorSimCreate(CARD, 0, LOW_LIM, HIGH_LIM, HOME_POS, 1, NUM_AXES)
iocsh.drvAsynMotorConfigure(INSTANCE .. CARD, INSTANCE, CARD, NUM_AXES)
-- TODO: just load the motor records here instead of substitutions file
iocsh.dbLoadTemplate("motorSim.substitutions", string.format(
    "P=%s,DTYP=%s,PORT=%s,DHLM=%s,DLLM=%s",
    P,
    "asynMotor",
    INSTANCE..CARD,
    HIGH_LIM,
    LOW_LIM)
)

-- //////////////////////////////////////////

-- Forward kinematics: compute TCP X position
function compute_pose_x()
    return L1 * math.cos(math.rad(A)) + L2*math.cos(math.rad(A+B))
end

-- Forward kinematics: compute TCP Y position
function compute_pose_y()
    return L1 * math.sin(math.rad(A)) + L2*math.sin(math.rad(A+B))
end

db.record("ai", P .. "J1") {
    EGU = "deg",
    INP = P .. "m1.RBV CP",
    PREC = 4,
}

db.record("ai", P .. "J2") {
    EGU = "deg",
    INP = P .. "m2.RBV CP",
    PREC = 4,
}

db.record("luascript", P .. "X") {
    INPA = P .. "J1.VAL CP",
    INPB = P .. "J2.VAL CP",
    EGU = "mm",
    PREC = 4,
    CODE = "@" .. STATE .. " compute_pose_x()",
}

db.record("luascript", P .. "Y") {
    INPA = P .. "J1.VAL CP",
    INPB = P .. "J2.VAL CP",
    EGU = "mm",
    PREC = 4,
    CODE = "@" .. STATE .. " compute_pose_y()",
}

db.record("ao", P .. "J1Target") {
    EGU = "deg",
    PREC = 4,
    OUT = P .. "m1.VAL PP"
}

db.record("ao", P .. "J2Target") {
    EGU = "deg",
    PREC = 4,
    OUT = P .. "m2.VAL PP"
}

db.record("dfanout", P .. "Stop") {
    DOL = 1,
    OUTA = P .. "m1.STOP PP",
    OUTB = P .. "m2.STOP PP"
}
