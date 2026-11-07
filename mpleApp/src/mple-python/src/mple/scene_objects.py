from dataclasses import dataclass
from typing import Literal

ShapeType = Literal["box", "sphere", "cylinder"]
Vector3 = tuple[float, float, float]


@dataclass
class Pose:
    position: Vector3
    orientation: Vector3


@dataclass
class SceneShape:
    shape_type: ShapeType
    pose: Pose
    dimensions: tuple[float, ...]


@dataclass
class SceneObject:
    name: str
    pose: Pose
    shapes: list[SceneShape]


@dataclass
class SceneObjects:
    objects: list[SceneObject]
    revision: int

    def __str__(self) -> str:
        lines = ["objects[]"]
        for obj in self.objects:
            lines.extend([
                f"    name: {obj.name}",
                f"    position[3]: {list(obj.pose.position)}",
                f"    orientation[3]: {list(obj.pose.orientation)}",
                "    shapes[]",
            ])
            for shape in obj.shapes:
                lines.extend([
                    f"        type: {shape.shape_type}",
                    f"        position[3]: {list(shape.pose.position)}",
                    f"        orientation[3]: {list(shape.pose.orientation)}",
                    f"        dimensions[{len(shape.dimensions)}]: {list(shape.dimensions)}",
                ])
        lines.append(f"revision: {self.revision}")
        return "\n".join(lines)
