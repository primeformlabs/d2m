"""Independent tri-dexel replay of fixed-axis Milling tool-tip moves.

The removal engine uses analytic ray/cutter intersections, independently of
the native Boolean model. Target comparison samples occupancy in part space.
It does not certify tolerances or manufacturing release.
"""
from __future__ import annotations

import math
import numpy as np

from .geometry import to_mesh
from .milling_replay import finite, validate_cutter
from .tool_library import to_sim_tool
from .toolpath import Toolpath


def replay_milling_independently(incoming, path: Toolpath, tool, *, resolution_mm: float = 0.5,
                                 target=None, tolerance_mm: float = 0.1):
    try:
        from src.simulation.tri_dexel import TriDexelModel
        from src.simulation.material_removal import MaterialRemovalEngine
        from src.simulation.kinematics import ToolPose
    except ImportError:
        from simulation.tri_dexel import TriDexelModel
        from simulation.material_removal import MaterialRemovalEngine
        from simulation.kinematics import ToolPose
    resolution = finite(resolution_mm, "independent replay resolution", positive=True)
    tolerance = finite(tolerance_mm, "target tolerance", positive=True)
    if path.metadata.get("coordinate_system") != "part_mm" or path.metadata.get("tool_axis", [0, 0, 1]) != [0, 0, 1]:
        raise ValueError("Independent Milling replay requires part millimetres and the positive Z cutter axis.")
    if (path.tool_type and path.tool_type != tool.tool_type) or (path.tool_diameter_mm and path.tool_diameter_mm != tool.diameter):
        raise ValueError("The selected cutter does not match the serialized Milling path.")
    shape = getattr(incoming, "_shape", incoming)
    mesh = to_mesh(shape)
    extent = mesh.bounds[1] - mesh.bounds[0]
    counts = [max(1, math.ceil(value / resolution)) for value in extent]
    if math.prod(counts) > 100000:
        raise ValueError("The independent replay exceeds its sample limit. Divide the process into smaller regions.")
    model = TriDexelModel.from_mesh(mesh, resolution=resolution)
    engine = MaterialRemovalEngine(model, linear_resolution=resolution / 2)
    selected_tool = to_sim_tool(tool)
    if selected_tool is None:
        raise ValueError("The independent Milling cutter model is unavailable.")
    cutting_positions = [move.position[2] for move in path.moves if move.move_type in {"feed", "plunge"}]
    if len(path.moves) > 10000:
        raise ValueError("The independent replay exceeds its move limit. Divide the process into smaller paths.")
    if not cutting_positions:
        raise ValueError("The independent Milling path has no cutting moves.")
    validate_cutter(tool, cutting_depth=max(0, mesh.bounds[1][2] - min(cutting_positions)))
    initial_volume, previous, cutting_moves = model.volume(), None, 0
    for move in path.moves:
        if not all(math.isfinite(value) for value in move.position):
            raise ValueError("The independent Milling path contains non-finite coordinates.")
        if move.move_type == "arc":
            raise ValueError("Independent Milling replay requires linearized arc moves.")
        pose = ToolPose(np.asarray(move.position, dtype=float), np.array([1.0, 0.0, 0.0, 0.0]))
        if move.move_type in {"feed", "plunge"}:
            if previous is None:
                raise ValueError("The independent cutting move requires a preceding tool position.")
            engine.remove_tool_swept(selected_tool, previous, pose)
            cutting_moves += 1
        previous = pose
    comparison = None
    if target is not None:
        target_shape = getattr(target, "_shape", target)
        expected = TriDexelModel.from_mesh(to_mesh(target_shape), resolution=resolution)
        overcut, undercut, samples = 0, 0, 0
        for ix in range(counts[0]):
            for iy in range(counts[1]):
                for iz in range(counts[2]):
                    point = mesh.bounds[0] + resolution * (np.array([ix, iy, iz]) + 0.5)
                    if np.any(point >= mesh.bounds[1]):
                        continue
                    actual_material = model.get_material_at(point)
                    target_material = expected.get_material_at(point)
                    samples += 1
                    overcut += int(target_material and not actual_material)
                    undercut += int(actual_material and not target_material)
        comparison = {"method": "tri_dexel_occupancy_samples", "status": "sampled_match" if not overcut and not undercut else "differences",
            "samples": samples, "overcut_samples": overcut, "undercut_samples": undercut,
            "resolution_mm": resolution, "requested_tolerance_mm": tolerance, "tolerance_verified": False}
    return model, {"method": "independent_tri_dexel_replay", "status": "requires_review", "cutting_moves": cutting_moves,
        "initial_volume_mm3": initial_volume, "final_volume_mm3": model.volume(),
        "removed_volume_mm3": initial_volume - model.volume(), "resolution_mm": resolution,
        "target_comparison": comparison, "manufacturing_ready": False,
        "reasons": ["The replay uses finite grid and motion sampling. Target tolerance requires further verification.",
            "Fixture, holder clearance, feed, and spindle qualification remain required."]}
