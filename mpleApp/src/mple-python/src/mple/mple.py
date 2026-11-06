#!/usr/bin/env python3
from p4p.client.thread import Context
from p4p import Value, Type
from math import pi
import numpy as np

from mple.scene_objects import Pose, SceneObject, SceneObjects


class Mple:
    def __init__(self, prefix: str, connection_timeout: float = 1.0):
        self.prefix = prefix
        self.ctxt = Context("pva")
        try:
            self.ctxt.get(f"{prefix}Status", timeout=connection_timeout)
        except TimeoutError:
            print("Timed out waiting to connect. Is an IOC with MPLE running?")

    def get_scene_objects(self, timeout: float = 5.0) -> SceneObjects:
        value = self.ctxt.get(f"{self.prefix}SceneObjects", timeout=timeout)
        objects = [
            SceneObject(
                name=obj["name"],
                shape_type=obj["type"],
                pose=Pose(
                    position=tuple(obj["position"]),
                    orientation=tuple(obj["orientation"]),
                ),
                shape_pose=Pose(
                    position=tuple(obj["shape_position"]),
                    orientation=tuple(obj["shape_orientation"]),
                ),
                dimensions=tuple(obj["dimensions"]),
            )
            for obj in value["objects"]
        ]
        return SceneObjects(objects=objects, revision=value["revision"])

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
        return result

    def execute(self, waypoints, speed, acceleration, timeout=300.0):
        ExecuteMsg = Value(Type([
            ('waypoints', 'ad'),
            ('speed', 'd'),
            ('acceleration', 'd'),
            ('timeout', 'd')
        ]), {
            'waypoints': waypoints,
            'speed': speed,
            'acceleration': acceleration,
            'timeout': timeout,
        })
        result = self.ctxt.rpc(f"{self.prefix}Execute", ExecuteMsg)
        return result

    def validate_and_execute(self, joint_waypoints, speed, acceleration):
        if (self.validate(joint_waypoints)["valid"]):
            self.execute(joint_waypoints, speed, acceleration)
        else:
            print("Validation failed. Aborting.")

    def plan(self, goal, start=[]):
        PlanMsg = Value(Type([
            ('start', 'ad'),
            ('goal', 'ad'),
        ]), {
            'start': start,
            'goal': goal,
        })

        result = self.ctxt.rpc(f"{self.prefix}Plan", PlanMsg)
        #  print(f"Valid: {result['valid']}")
        #  print(f"Description: {result['description']}")
        return result

    def compute_fk(self, joints):
        ValidateMsg = Value(Type([
            ('joints', 'ad'),
        ]), {
            'joints': joints,
        })

        result = self.ctxt.rpc(f"{self.prefix}ComputeFK", ValidateMsg)
        #  print(f"Valid: {result['valid']}")
        #  print(f"Description: {result['description']}")
        return result

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

