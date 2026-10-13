"""Target-geometry comparison helpers for SubCAD.

This module keeps SubCAD's public comparison API small while reusing the
localized mesh comparison tools from ``src.simulation.mesh_compare``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any


def compare_to_target(candidate: Any, target: Any, tolerance_mm: float = 0.25) -> dict:
    """Compare a candidate Stock/shape/mesh against a target geometry.

    ``target`` may be another ``Stock``, a CadQuery workplane, a trimesh mesh,
    or a path to STEP/STL/OBJ-like mesh data.  The returned dict is JSON-ready
    and summarizes overcut/undercut/deviation risk.
    """
    try:
        from src.simulation.mesh_compare import compare_meshes
    except ImportError:
        from simulation.mesh_compare import compare_meshes

    candidate_mesh = _as_mesh(candidate)
    target_mesh = _as_mesh(target)
    if candidate_mesh is None:
        raise ValueError("Candidate geometry could not be converted to a mesh")
    if target_mesh is None:
        raise ValueError("Target geometry could not be converted to a mesh")

    raw = compare_meshes(
        target_mesh,
        candidate_mesh,
        methods=["sdf", "slice"],
        sample_distance=max(float(tolerance_mm), 0.01),
    )
    sdf = raw.get("sdf") or {}
    clusters = list(raw.get("clusters") or [])
    overcut_regions = [c for c in clusters if c.get("type") == "overcut"]
    undercut_regions = [c for c in clusters if c.get("type") == "undercut"]
    max_overcut = float(sdf.get("max_overcut", 0.0) or 0.0)
    max_undercut = abs(float(sdf.get("max_undercut", 0.0) or 0.0))
    rms_error = float(sdf.get("rms_error", 0.0) or 0.0)
    score = float(raw.get("score", 0.0) or 0.0)

    status = "pass"
    if max(max_overcut, max_undercut, rms_error) > tolerance_mm:
        status = "fail"
    elif score < 95.0:
        status = "warning"

    return {
        "status": status,
        "tolerance_mm": float(tolerance_mm),
        "score": score,
        "rms_error_mm": round(rms_error, 6),
        "max_overcut_mm": round(max_overcut, 6),
        "max_undercut_mm": round(max_undercut, 6),
        "overcut_vertices": int(sdf.get("overcut_vertices", 0) or 0),
        "undercut_vertices": int(sdf.get("undercut_vertices", 0) or 0),
        "overcut_regions": _compact_regions(overcut_regions),
        "undercut_regions": _compact_regions(undercut_regions),
        "slice": _compact_slice(raw.get("slice") or {}),
        "feedback": raw.get("feedback", ""),
    }


def _as_mesh(source: Any):
    if source is None:
        return None
    if hasattr(source, "to_mesh"):
        return source.to_mesh()
    if hasattr(source, "_shape"):
        from .geometry import to_mesh

        return to_mesh(source._shape)
    if hasattr(source, "vertices") and hasattr(source, "faces"):
        return source
    if hasattr(source, "val"):
        from .geometry import to_mesh

        return to_mesh(source)
    if isinstance(source, (str, Path)):
        return _mesh_from_path(Path(source))
    return None


def _mesh_from_path(path: Path):
    suffix = path.suffix.lower()
    if suffix in {".step", ".stp"}:
        from .geometry import import_step, to_mesh

        return to_mesh(import_step(str(path)))

    import trimesh

    loaded = trimesh.load(str(path))
    if isinstance(loaded, trimesh.Scene):
        return loaded.dump(concatenate=True)
    return loaded


def _compact_regions(regions: list[dict], limit: int = 8) -> list[dict]:
    compact = []
    for region in regions[:limit]:
        compact.append({
            "type": region.get("type"),
            "center": _round_list(region.get("center", [])),
            "max_deviation_mm": round(float(region.get("max_deviation", 0.0) or 0.0), 6),
            "mean_deviation_mm": round(float(region.get("mean_deviation", 0.0) or 0.0), 6),
            "vertex_count": int(region.get("vertex_count", 0) or 0),
            "bbox": [_round_list(v) for v in region.get("bbox", [])],
        })
    return compact


def _compact_slice(slice_result: dict) -> dict:
    return {
        "max_diff_z": round(float(slice_result.get("max_diff_z", 0.0) or 0.0), 6),
        "max_diff_pct": round(float(slice_result.get("max_diff_pct", 0.0) or 0.0), 6),
        "problem_slices": slice_result.get("problem_slices", [])[:8],
    }


def _round_list(values) -> list[float]:
    return [round(float(v), 6) for v in values]
