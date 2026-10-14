"""Neutral toolpath primitives for SubCAD shop-floor handoff.

The classes here intentionally avoid controller-specific G-code details.  They
capture enough motion intent for simulation, estimating, and JSON exchange while
leaving post-processing to a later layer.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from typing import Any, Iterable, Optional, Sequence, Tuple


Position = Tuple[float, float, float]
MoveType = str


def _position_tuple(position: Sequence[float]) -> Position:
    if len(position) != 3:
        raise ValueError("Toolpath positions must be 3D (x, y, z).")
    return (float(position[0]), float(position[1]), float(position[2]))


@dataclass
class ToolpathMove:
    """One neutral tool movement.

    move_type is one of rapid/feed/plunge/retract/dwell/arc.  Arc moves are a
    placeholder representation with optional center and direction metadata.
    """

    move_type: MoveType
    position: Position
    feed: Optional[float] = None
    spindle_rpm: Optional[float] = None
    coolant: Optional[bool] = None
    dwell_seconds: float = 0.0
    arc_center: Optional[Position] = None
    arc_direction: Optional[str] = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.move_type = self.move_type.lower()
        valid = {"rapid", "feed", "plunge", "retract", "dwell", "arc"}
        if self.move_type not in valid:
            raise ValueError(
                f"Unsupported move_type {self.move_type!r}; expected one of {sorted(valid)}"
            )
        self.position = _position_tuple(self.position)
        if self.arc_center is not None:
            self.arc_center = _position_tuple(self.arc_center)

    def distance_from(self, previous: Optional["ToolpathMove"]) -> float:
        if previous is None or self.move_type == "dwell":
            return 0.0
        if self.move_type == "arc" and self.arc_center is not None:
            return self._arc_distance(previous.position)
        return math.dist(previous.position, self.position)

    def time_minutes_from(
        self,
        previous: Optional["ToolpathMove"],
        default_feed: float,
        rapid_rate: float,
    ) -> float:
        if self.move_type == "dwell":
            return max(self.dwell_seconds, 0.0) / 60.0
        rate = rapid_rate if self.move_type in {"rapid", "retract"} else self.feed
        rate = rate or default_feed
        if rate <= 0:
            return 0.0
        return self.distance_from(previous) / rate

    def to_dict(self) -> dict[str, Any]:
        data: dict[str, Any] = {
            "type": self.move_type,
            "position": list(self.position),
        }
        if self.feed is not None:
            data["feed_mm_min"] = self.feed
        if self.spindle_rpm is not None:
            data["spindle_rpm"] = self.spindle_rpm
        if self.coolant is not None:
            data["coolant"] = self.coolant
        if self.dwell_seconds:
            data["dwell_seconds"] = self.dwell_seconds
        if self.arc_center is not None:
            data["arc_center"] = list(self.arc_center)
        if self.arc_direction is not None:
            data["arc_direction"] = self.arc_direction
        if self.metadata:
            data["metadata"] = self.metadata
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ToolpathMove":
        return cls(
            move_type=data.get("type") or data.get("move_type"),
            position=data["position"],
            feed=data.get("feed_mm_min", data.get("feed")),
            spindle_rpm=data.get("spindle_rpm"),
            coolant=data.get("coolant"),
            dwell_seconds=data.get("dwell_seconds", 0.0),
            arc_center=data.get("arc_center"),
            arc_direction=data.get("arc_direction"),
            metadata=dict(data.get("metadata", {})),
        )

    def _arc_distance(self, start: Position) -> float:
        assert self.arc_center is not None
        sx, sy, _ = start
        ex, ey, _ = self.position
        cx, cy, _ = self.arc_center
        radius = math.dist((sx, sy), (cx, cy))
        if radius <= 0:
            return math.dist(start, self.position)
        a0 = math.atan2(sy - cy, sx - cx)
        a1 = math.atan2(ey - cy, ex - cx)
        delta = a1 - a0
        if self.arc_direction == "cw" and delta > 0:
            delta -= math.tau
        elif self.arc_direction == "ccw" and delta < 0:
            delta += math.tau
        return abs(delta) * radius + abs(self.position[2] - start[2])


@dataclass
class Toolpath:
    """Neutral, JSON-serializable path for one machining operation."""

    operation: str
    moves: list[ToolpathMove] = field(default_factory=list)
    safe_z: float = 5.0
    tool_type: Optional[str] = None
    tool_diameter_mm: Optional[float] = None
    feed_mm_min: Optional[float] = None
    spindle_rpm: Optional[float] = None
    coolant: bool = False
    rapid_rate_mm_min: float = 5000.0
    metadata: dict[str, Any] = field(default_factory=dict)

    def __iter__(self):
        return iter(self.moves)

    def __len__(self) -> int:
        return len(self.moves)

    def __getitem__(self, index: int) -> ToolpathMove:
        return self.moves[index]

    def append(self, move: ToolpathMove) -> None:
        self.moves.append(move)

    @property
    def length_mm(self) -> float:
        return sum(
            move.distance_from(previous)
            for previous, move in zip([None] + self.moves[:-1], self.moves)
        )

    @property
    def estimated_time_min(self) -> float:
        feed = self.feed_mm_min or 100.0
        return sum(
            move.time_minutes_from(previous, feed, self.rapid_rate_mm_min)
            for previous, move in zip([None] + self.moves[:-1], self.moves)
        )

    def to_positions(self) -> list[list[float]]:
        return [list(move.position) for move in self.moves]

    def to_dict(self) -> dict[str, Any]:
        return {
            "operation": self.operation,
            "safe_z": self.safe_z,
            "tool_type": self.tool_type,
            "tool_diameter_mm": self.tool_diameter_mm,
            "feed_mm_min": self.feed_mm_min,
            "spindle_rpm": self.spindle_rpm,
            "rapid_rate_mm_min": self.rapid_rate_mm_min,
            "coolant": self.coolant,
            "length_mm": self.length_mm,
            "estimated_time_min": self.estimated_time_min,
            "moves": [move.to_dict() for move in self.moves],
            "positions": self.to_positions(),
            "metadata": self.metadata,
        }

    def to_json(self, **kwargs: Any) -> str:
        return json.dumps(self.to_dict(), **kwargs)

    @classmethod
    def from_moves(
        cls,
        operation: str,
        moves: Iterable[ToolpathMove],
        **kwargs: Any,
    ) -> "Toolpath":
        return cls(operation=operation, moves=list(moves), **kwargs)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Toolpath":
        return cls(
            operation=data["operation"],
            moves=[ToolpathMove.from_dict(m) for m in data.get("moves", [])],
            safe_z=data.get("safe_z", 5.0),
            tool_type=data.get("tool_type"),
            tool_diameter_mm=data.get("tool_diameter_mm"),
            feed_mm_min=data.get("feed_mm_min"),
            spindle_rpm=data.get("spindle_rpm"),
            coolant=data.get("coolant", False),
            rapid_rate_mm_min=data.get("rapid_rate_mm_min", 5000.0),
            metadata=dict(data.get("metadata", {})),
        )
