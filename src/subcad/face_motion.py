"""Bind supported face machining paths to the incoming stock coordinates."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .toolpath import Toolpath


@dataclass(frozen=True)
class FaceMotionFrame:
    bounds: dict[str, float]
    face: str

    def __post_init__(self) -> None:
        if self.face not in {">X", "<X", ">Y", "<Y", ">Z", "<Z"}:
            raise ValueError("The toolpath requires a cardinal stock face.")
        for axis in "xyz":
            low, high = self.bounds[f"{axis}_min"], self.bounds[f"{axis}_max"]
            if not math.isfinite(low) or not math.isfinite(high) or high <= low:
                raise ValueError("The incoming stock bounds must be finite and ordered.")

    @property
    def axes(self) -> tuple[int, int, int]:
        return {"X": (1, 2, 0), "Y": (0, 2, 1), "Z": (0, 1, 2)}[self.face[1]]

    @property
    def sign(self) -> int:
        return 1 if self.face[0] == ">" else -1

    @property
    def surface(self) -> float:
        return self.bounds[f"{self.face[1].lower()}_{'max' if self.sign > 0 else 'min'}"]

    @property
    def footprint(self) -> tuple[float, float, float, float]:
        u, v, _ = self.axes
        return tuple(self.bounds[f"{'xyz'[index]}_{limit}"]
                     for index in (u, v) for limit in ("min", "max"))

    @property
    def tool_axes(self) -> dict[str, list[int]]:
        u, _, normal = self.axes
        x = [int(index == u) for index in range(3)]
        z = [self.sign * int(index == normal) for index in range(3)]
        y = [z[1] * x[2] - z[2] * x[1],
             z[2] * x[0] - z[0] * x[2],
             z[0] * x[1] - z[1] * x[0]]
        return {"x_axis": x, "y_axis": y, "z_axis": z}

    def position(self, local: tuple[float, float, float]) -> tuple[float, float, float]:
        u, v, normal = self.axes
        point = [0.0, 0.0, 0.0]
        point[u], point[v] = local[0], local[1]
        point[normal] = self.surface + self.sign * local[2]
        return tuple(point)

    def bind(self, path: Toolpath) -> Toolpath:
        """Positions use stock axes; the separate cutter frame is right handed."""
        for move in path.moves:
            if move.arc_center is not None:
                raise ValueError("Stock binding requires linear moves with a stated chord error.")
            move.position = self.position(move.position)
        path.metadata.update({
            "coordinate_system": "stock_mm",
            "tool_origin": "cutting_tip",
            "tool_frame": self.tool_axes,
            "face_selector": self.face,
            "incoming_bounds_mm": dict(self.bounds),
            "clearance_mm": path.safe_z,
            "safe_z_is_clearance": True,
            "verification_status": "not_verified",
            "source": "cam_intent",
        })
        return path


def rounded_contour(half_x: float, half_y: float, radius: float,
                    chord_error: float) -> list[tuple[float, float]]:
    """Return a closed cutter-centre contour with inward circular arc chords."""
    radius = min(max(radius, 0.0), half_x, half_y)
    if radius <= 1e-9:
        return [(half_x, -half_y), (half_x, half_y), (-half_x, half_y),
                (-half_x, -half_y), (half_x, -half_y)]
    angle = 2 * math.acos(max(-1.0, min(1.0, 1 - chord_error / radius)))
    count = max(1, math.ceil((math.pi / 2) / angle))
    points = []
    for cx, cy, start_angle in (
        (half_x - radius, -half_y + radius, -math.pi / 2),
        (half_x - radius, half_y - radius, 0),
        (-half_x + radius, half_y - radius, math.pi / 2),
        (-half_x + radius, -half_y + radius, math.pi),
    ):
        for step in range(count + 1):
            theta = start_angle + step * math.pi / (2 * count)
            points.append((cx + radius * math.cos(theta), cy + radius * math.sin(theta)))
    points.append(points[0])
    return points
