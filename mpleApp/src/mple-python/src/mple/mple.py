#!/usr/bin/env python3
from p4p.client.thread import Context
from p4p import Value, Type
from math import pi
import numpy as np

class Mple:
    def __init__(self, prefix: str, connection_timeout: float = 1.0):
        self.prefix = prefix
        self.ctxt = Context("pva")
        try:
            self.ctxt.get(f"{prefix}Status", timeout=connection_timeout)
        except TimeoutError:
            print("Timed out waiting to connect. Is an IOC with MPLE running?")

    def validate(self, joint_waypoints) -> bool:
        ValidateMsg = Value(Type([
            ('joint_waypoints', 'ad'),
            ('cartesian_waypoints', 'ad'),
            ('resolution', 'd'),
        ]), {
            'joint_waypoints': joint_waypoints,
            'resolution': 3.1415
        })

        result = self.ctxt.rpc(f"{self.prefix}Validate", ValidateMsg)
        #  print(f"Valid: {result['valid']}")
        #  print(f"Description: {result['description']}")
        #  return result["valid"]
        return result

    def execute(self, joint_waypoints):
        ExecuteMsg = Value(Type([
            ('joint_waypoints', 'ad'),
            ('speed', 'd'),
            ('acceleration', 'd')
        ]), {
            'joint_waypoints': joint_waypoints,
            'speed': pi/3,
            'acceleration': pi,
        })
        result = self.ctxt.rpc(f"{self.prefix}Execute", ExecuteMsg)
        return result
        #  print(f"Error Code: {result["error_code"]}")
        #  print(f"Description: {result["description"]}")


    def validate_and_execute(self,joint_waypoints):
        if (self.validate(joint_waypoints)["valid"]):
            self.execute(joint_waypoints)
        else:
            print("Validation failed. Aborting.")


    def compute_ik(self, poses):
        ValidateMsg = Value(Type([
            ('cartesian_waypoints', 'ad'),
        ]), {
            'cartesian_waypoints': poses,
        })

        result = self.ctxt.rpc(f"{self.prefix}ComputeIK", ValidateMsg)
        #  print(f"Valid: {result['valid']}")
        #  print(f"Description: {result['description']}")
        return result

