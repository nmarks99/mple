#!/usr/bin/env python3
from p4p.client.thread import Context
from p4p import Value, Type
from math import pi

ctxt = Context("pva")
PREFIX = "mpleTest:"


def validate(joint_waypoints) -> bool:
    ValidateMsg = Value(Type([
        ('joint_waypoints', 'ad'),
        ('cartesian_waypoints', 'ad'),
        ('resolution', 'd'),
    ]), {
        'joint_waypoints': joint_waypoints,
        'resolution': 3.1415
    })

    result = ctxt.rpc(f"{PREFIX}Validate", ValidateMsg)
    print(f"Valid: {result['valid']}")
    print(f"Description: {result['description']}")
    return result["valid"]

def execute(joint_waypoints):
    ExecuteMsg = Value(Type([
        ('joint_waypoints', 'ad'),
        ('speed', 'd'),
        ('acceleration', 'd')
    ]), {
        'joint_waypoints': joint_waypoints,
        'speed': pi/3,
        'acceleration': pi,
    })
    result = ctxt.rpc(f"{PREFIX}Execute", ExecuteMsg)
    print(f"Error Code: {result["error_code"]}")
    print(f"Description: {result["description"]}")


def validate_and_execute(joint_waypoints):
    if (validate(joint_waypoints)):
        execute(joint_waypoints)
    else:
        print("Validation failed. Aborting.")


wps = [
    0.0, 0.0,
    pi/2, pi/2,
    pi/6, pi/2,
    -pi/3, -pi/3
]
validate_and_execute(wps)
