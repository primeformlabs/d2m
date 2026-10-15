"""Postprocessor foundation for SubCAD process plans.

The only built-in implementation is intentionally debug-only.  Production
machine/controller posts should subclass ``PostProcessor`` and explicitly
remove the non-production guard once they are validated.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


NON_PRODUCTION_NOTICE = (
    "DEBUG POST ONLY - NOT FOR PRODUCTION / VERIFY IN CAM BEFORE CUTTING"
)


@dataclass
class PostProcessor:
    """Base postprocessor interface for process-plan JSON v1."""

    controller: str = "generic_3axis_debug"
    machine: str = "generic_3axis_mill"
    safe_mode: bool = True
    production_certified: bool = False

    def postprocess(self, plan: Any) -> str:
        """Return posted text for *plan*.

        The base implementation only supports the debug controller.
        """
        if self.controller != "generic_3axis_debug":
            raise ValueError(
                f"No production postprocessor registered for controller "
                f"{self.controller!r}; use generic_3axis_debug for debug output."
            )
        return Generic3AxisDebugPost(
            controller=self.controller,
            machine=self.machine,
            safe_mode=self.safe_mode,
            production_certified=self.production_certified,
        ).postprocess(plan)


class Generic3AxisDebugPost(PostProcessor):
    """Readable non-production 3-axis post for tests and review."""

    def postprocess(self, plan: Any) -> str:
        plan_dict = _plan_dict(plan)
        lines = [
            f"({NON_PRODUCTION_NOTICE})",
            f"(Controller: {self.controller})",
            f"(Machine: {self.machine})",
            f"(Schema: {plan_dict.get('schema_version', 'unknown')})",
            "G21",
            "G90",
        ]
        if self.safe_mode:
            lines.extend(["G17", "G40", "G49", "G80"])

        setups = plan_dict.get("setups") or {}
        for op in plan_dict.get("operations", []):
            seq = op.get("sequence_number", "?")
            name = op.get("operation", "?")
            setup_name = op.get("setup") or "unspecified"
            setup = setups.get(setup_name, {}) if isinstance(setups, dict) else {}
            work_offset = setup.get("work_offset") if isinstance(setup, dict) else None
            lines.append("")
            lines.append(f"(OP {seq}: {name}; setup={setup_name}; DEBUG ONLY)")
            if op.get("process") == "turn" or str(name).startswith("turn_"):
                lines.append("(Turning requires a lathe postprocessor; no motion emitted.)")
                continue
            if work_offset:
                lines.append(str(work_offset))
            tool = op.get("tool") or {}
            tool_label = tool.get("label") or op.get("tool_type", "unknown_tool")
            lines.append(f"(Tool: {tool_label})")
            toolpath = op.get("toolpath")
            if toolpath:
                lines.extend(_debug_moves(toolpath, op))
            else:
                lines.append("(No neutral toolpath; no motion emitted.)")

        lines.extend(["", "M30", f"({NON_PRODUCTION_NOTICE})"])
        return "\n".join(lines)


def postprocess(
    plan: Any,
    controller: str = "generic_3axis_debug",
    machine: str = "generic_3axis_mill",
    safe_mode: bool = True,
) -> str:
    """Convenience postprocess function."""
    return PostProcessor(
        controller=controller,
        machine=machine,
        safe_mode=safe_mode,
    ).postprocess(plan)


def _plan_dict(plan: Any) -> dict:
    if hasattr(plan, "to_dict"):
        return plan.to_dict()
    if isinstance(plan, dict):
        return plan
    raise TypeError("postprocess expects a ProcessPlan or process-plan dict")


def _debug_moves(toolpath: Any, op: dict) -> list[str]:
    if hasattr(toolpath, "to_dict"):
        toolpath = toolpath.to_dict()
    if isinstance(toolpath, dict):
        moves = toolpath.get("moves") or toolpath.get("positions") or []
    elif isinstance(toolpath, list):
        moves = toolpath
    else:
        moves = []
    feed = _feed(op)
    lines = []
    for move in moves:
        if isinstance(move, dict):
            pos = move.get("position")
            mtype = str(move.get("type", move.get("move_type", "feed"))).lower()
            feed = move.get("feed_rate_mm_per_min", move.get("feed", feed))
        elif isinstance(move, (list, tuple)) and len(move) >= 3:
            pos = move[:3]
            mtype = "feed"
        else:
            continue
        if not pos or len(pos) < 3:
            continue
        code = "G0" if mtype in {"rapid", "retract"} else "G1"
        if mtype == "arc":
            code = "G3"
        line = f"{code} X{float(pos[0]):.4f} Y{float(pos[1]):.4f} Z{float(pos[2]):.4f}"
        if code in {"G1", "G3"} and feed:
            line += f" F{float(feed):.1f}"
        lines.append(line)
    return lines or ["(Neutral toolpath contained no positioned moves.)"]


def _feed(op: dict):
    feeds = op.get("feeds_speeds") if isinstance(op.get("feeds_speeds"), dict) else {}
    return feeds.get("feed_rate_mm_per_min", feeds.get("feed_rate_mm_min"))


if __name__ == "__main__":
    sample = {
        "schema_version": "subcad.shop_floor.v1",
        "operations": [
            {
                "sequence_number": 1,
                "operation": "debug",
                "toolpath": {"moves": [{"type": "rapid", "position": [0, 0, 5]}]},
            }
        ],
    }
    output = postprocess(sample)
    assert NON_PRODUCTION_NOTICE in output
    assert "G21" in output
    print("postprocessor self-test passed")
