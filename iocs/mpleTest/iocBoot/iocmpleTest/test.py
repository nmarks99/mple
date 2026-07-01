#!/usr/bin/env python3
from p4p.client.thread import Context
from p4p import Value, Type
ctxt = Context("pva")
PREFIX = "mpleTest:"

ValidateMsg = Value(Type([
    ('joint_waypoints', 'ad'),
    ('cartesian_waypoints', 'ad'),
    ('resolution', 'd'),
]), {
    'joint_waypoints': [1.0, 2.0],
    'resolution': 3.1415
})

ExecuteMsg = Value(Type([
    ('joint_waypoints', 'ad'),
    ('speeds', 'ad'),
    ('accelerations', 'ad')]), {
    'joint_waypoints': [90.0, 180.0],
    'speeds': [1.0, 1.0],
    'accelerations': [0.12, 0.34],
})

result = ctxt.rpc(f"mpleTest:Validate", ValidateMsg)
print(f"Valid: {result["valid"]}")
print(f"Description: {result["description"]}")

if result["valid"]:
    result = ctxt.rpc(f"mpleTest:Execute", ExecuteMsg)
    print(f"Code: {result["error_code"]}")
    print(f"Description: {result["description"]}")
else:
    print("Execute Failed!")
