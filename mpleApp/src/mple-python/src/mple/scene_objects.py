from dataclasses import dataclass
from typing import Literal

ShapeType = Literal["box", "sphere", "cylinder"]
Vector3 = tuple[float, float, float]

@dataclass
class Pose:
    position: Vector3
    orientation: Vector3


@dataclass
class SceneObject:
    name: str
    shape_type: ShapeType
    pose: Pose
    shape_pose: Pose
    dimensions: tuple[float, ...]


@dataclass
class SceneObjects:
    objects: list[SceneObject]
    revision: int
