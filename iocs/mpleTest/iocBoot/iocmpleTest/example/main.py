#!/usr/bin/env python3
from p4p.client.thread import Context
from p4p import Value, Type
from math import pi

ctxt = Context("pva")
PREFIX = "mpleTest:"

ValidateMsg = Value(Type([
    ('joint_waypoints', 'ad'),
    ('cartesian_waypoints', 'ad'),
    ('resolution', 'd'),
]), {
    'joint_waypoints': [1.57, 0.0],
    'resolution': 3.1415
})

ExecuteMsg = Value(Type([
    ('joint_waypoints', 'ad'),
    ('speed', 'd'),
    ('acceleration', 'd')
]), {
    'joint_waypoints': [
        pi/2, 0.0,
        #  pi/3, pi/3,
        #  pi/2, -pi/3,
        #  0.0, -pi/3,
        #  -pi/8, -pi/2
    ],
    'speed': pi/3,
    'acceleration': pi,
})

result = ctxt.rpc(f"mpleTest:Validate", ValidateMsg)
print(f"Valid: {result['valid']}")
print(f"Description: {result['description']}")
# TODO: only continue if result["valid"]

result = ctxt.rpc(f"mpleTest:Execute", ExecuteMsg)
print(f"Error Code: {result["error_code"]}")
print(f"Description: {result["description"]}")
