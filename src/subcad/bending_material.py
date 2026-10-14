"""Retained sheet-mesh bending from explicit process steps.

This is a radiused kinematic preview. It does not qualify material behaviour,
springback, contact, tooling, or manufacturing tolerances.
"""
from copy import deepcopy
from dataclasses import dataclass
import math

import numpy as np
import trimesh
from scipy.spatial import cKDTree


MODEL_REVISION = "retained-sheet-bending/v1"
MAX_TRIANGLES = 200_000
STL_SEAM_TOLERANCE_MM = 0.00002
LIMITATIONS = [
    "Bend deformation uses a radiused kinematic model of the actual incoming material.",
    "Material behaviour, springback, tooling contact, and manufacturing tolerances remain unverified.",
    "STL facet edges use a 0.00002 mm numerical seam limit. This limit does not qualify a manufacturing tolerance.",
    "The kinematic bend acts across the sheet on the recorded bend axis. Finite flange boundaries still need qualification.",
]


@dataclass
class BendingMaterialPass:
    step_id: str
    before: trimesh.Trimesh
    after: trimesh.Trimesh
    report: dict


@dataclass
class BendingMaterialReplay:
    material: trimesh.Trimesh
    passes: list[BendingMaterialPass]
    report: dict


def _number(value, name, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"The {name} must be a finite number.")
    if positive and value <= 0:
        raise ValueError(f"The {name} must be positive.")
    return float(value)


def _vector(value, name):
    if not isinstance(value, dict) or set(value) != {"x", "y", "z"}:
        raise ValueError(f"The {name} requires explicit XYZ coordinates.")
    return np.array([_number(value[key], name) for key in ("x", "y", "z")])


def _unit(value, name):
    length = np.linalg.norm(value)
    if length < 1e-9:
        raise ValueError(f"The {name} has no usable direction.")
    return value / length


def _steps(plan):
    if not isinstance(plan, dict) or not isinstance(plan.get("flat_blank"), dict):
        raise ValueError("Bending requires explicit flat-blank dimensions and ordered steps.")
    blank = plan["flat_blank"]
    dimensions = [_number(blank.get(key), key, positive=True) for key in
                  ("length_mm", "width_mm", "thickness_mm")]
    raw = plan.get("steps")
    if not isinstance(raw, list) or not 1 <= len(raw) <= 50 or plan.get("partial_sequence") or plan.get("unsequenced_bends"):
        raise ValueError("Bending requires a complete ordered step sequence.")
    result, ids = [], set()
    for index, step in enumerate(raw):
        if not isinstance(step, dict) or not isinstance(step.get("id"), str) or not step["id"]:
            raise ValueError("Each Bending step requires an explicit identity.")
        if step["id"] in ids or type(step.get("order")) is not int or step["order"] != index + 1:
            raise ValueError("Bending step identities and order must be unique and contiguous.")
        ids.add(step["id"])
        angle = _number(step.get("angle_deg"), "bend angle")
        signed = _number(step.get("signed_fold_angle_deg"), "signed fold angle")
        if not 1 <= abs(angle) < 180 or abs(abs(signed) - abs(angle)) > 1e-6:
            raise ValueError("The signed fold angle must match the required bend angle.")
        radius = _number(step.get("inside_radius_mm"), "inside radius")
        if radius < 0:
            raise ValueError("The inside radius cannot be negative.")
        line = step.get("bend_line") or {}
        start, end = _vector(line.get("start_mm"), "bend start"), _vector(line.get("end_mm"), "bend end")
        normal = _unit(_vector(line.get("normal_mm"), "sheet normal"), "sheet normal")
        if max(abs(start[1]), abs(end[1])) > 1e-6 or not np.allclose(normal, [0, 1, 0], atol=1e-6):
            raise ValueError("The bend line must use the flat XZ sheet frame and positive Y normal.")
        axis = _unit(end - start, "bend line")
        if step.get("moving_side") not in {"positive", "negative"}:
            raise ValueError("The Bending step requires an explicit moving side.")
        if step.get("direction_convention") == "sheet_face" or (step.get("tooling") or {}).get("source") == "step_classifier":
            if step.get("direction") not in {"up", "down"}:
                raise ValueError("The classifier bend requires an explicit sheet-face direction.")
            expected = (1 if step["direction"] == "up" else -1) * (1 if step["moving_side"] == "negative" else -1) * abs(angle)
            if abs(signed - expected) > 1e-6:
                raise ValueError("The signed fold angle contradicts the classifier sheet-face direction.")
        support = step.get("support_mode", "balanced")
        if support not in {"fixed", "balanced"}:
            raise ValueError("The Bending support mode is unavailable.")
        ratio = 0.5 if step.get("moving_ratio") is None else _number(step["moving_ratio"], "moving ratio")
        if not 0.5 <= ratio <= 0.9:
            raise ValueError("The moving ratio must be between 0.5 and 0.9.")
        result.append({"id": step["id"], "start": start, "end": end, "axis": axis,
            "side": _unit(np.cross(axis, normal), "moving direction"),
            "moving_sign": 1 if step["moving_side"] == "positive" else -1,
            "angle": math.radians(signed), "radius": radius + dimensions[2] / 2,
            "fixed": support == "fixed", "ratio": ratio})
    return dimensions, result


def _mesh(triangles):
    triangles = np.asarray(triangles, dtype=np.float32)
    vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    parents = np.arange(len(vertices))
    def root(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index
    # Native STL seams can use slightly different facet coordinates. Bound every
    # join; do not fill an open surface or generate replacement material.
    for left, right in cKDTree(vertices).query_pairs(STL_SEAM_TOLERANCE_MM, output_type="ndarray"):
        a, b = root(left), root(right)
        if a != b:
            parents[max(a, b)] = min(a, b)
    representatives = np.array([root(i) for i in range(len(vertices))])
    displacement = float(np.linalg.norm(vertices - vertices[representatives], axis=1).max())
    if displacement > STL_SEAM_TOLERANCE_MM:
        raise ValueError("The STL seam coordinates exceed the declared Bending precision limit.")
    faces = representatives[inverse].reshape(-1, 3)
    distinct = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    result = trimesh.Trimesh(vertices=vertices, faces=faces[distinct], process=False)
    result.remove_unreferenced_vertices()
    result.metadata["stl_precision"] = {"seam_tolerance_mm": STL_SEAM_TOLERANCE_MM,
        "maximum_vertex_join_mm": displacement, "removed_collapsed_facets": int((~distinct).sum())}
    return result


def _subdivide(triangles, steps):
    # Compute shared-edge intersections in double precision. Rounding each cut
    # to STL precision separates the same edge after subsequent subdivision.
    polygons = [polygon.astype(np.float64) for polygon in triangles]
    for step in steps:
        band = max(step["radius"] * abs(step["angle"]), 5)
        for offset in np.linspace(-band, band, 49):
            lengths = np.fromiter((len(p) for p in polygons), dtype=np.int64, count=len(polygons))
            starts = np.r_[0, np.cumsum(lengths)[:-1]]
            distances = (np.concatenate(polygons) - step["start"]) @ step["side"] - offset
            low, high = np.minimum.reduceat(distances, starts), np.maximum.reduceat(distances, starts)
            crossing = set(np.flatnonzero((low < -1e-8) & (high > 1e-8)))
            if not crossing:
                continue
            next_polygons = []
            for index, polygon in enumerate(polygons):
                if index not in crossing:
                    next_polygons.append(polygon)
                    continue
                source_distances = distances[starts[index]:starts[index] + lengths[index]]
                for sign in (-1, 1):
                    clipped = []
                    for i, point in enumerate(polygon):
                        j = (i + 1) % len(polygon)
                        a, b = source_distances[i] * sign, source_distances[j] * sign
                        if a >= -1e-8:
                            clipped.append(point)
                        if (a > 1e-8 and b < -1e-8) or (a < -1e-8 and b > 1e-8):
                            clipped.append(point + (polygon[j] - point) * (a / (a - b)))
                    if len(clipped) >= 3:
                        next_polygons.append(np.array(clipped))
            polygons = next_polygons
            if len(polygons) > MAX_TRIANGLES:
                raise ValueError("The actual material exceeds the Bending subdivision limit.")
    output = []
    for polygon in polygons:
        for i in range(1, len(polygon) - 1):
            if np.dot(np.cross(polygon[i] - polygon[0], polygon[i + 1] - polygon[0]),
                      np.cross(polygon[i] - polygon[0], polygon[i + 1] - polygon[0])) > 1e-20:
                output.append([polygon[0], polygon[i], polygon[i + 1]])
    if not output or len(output) > MAX_TRIANGLES:
        raise ValueError("The actual material exceeds the Bending triangle limit.")
    return np.asarray(output, dtype=np.float32)


def _rotate(points, axis, angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    return points * cosine + np.cross(axis, points) * sine + np.outer(points @ axis, axis) * (1 - cosine)


def _apply(points, original, step, frame):
    start, end, reference = frame
    axis = _unit(end - start, "formed bend line")
    direction = reference - start
    direction -= axis * np.dot(direction, axis)
    direction = _unit(direction, "formed moving direction")
    distances = ((original - step["start"]) @ step["side"]) * step["moving_sign"]
    along = np.outer((points - start) @ axis, axis) + start
    result = points.copy()
    masks = [(distances >= 0, direction, step["angle"], np.maximum(distances, 0))]
    if not step["fixed"]:
        masks = [(distances >= 0, direction, step["angle"] * step["ratio"], np.abs(distances)),
                 (distances < 0, -direction, -step["angle"] * (1 - step["ratio"]), np.abs(distances))]
    for mask, panel_direction, angle, distance in masks:
        radius = step["radius"]
        arc = radius * abs(angle)
        perpendicular = np.cross(axis, panel_direction) * (1 if angle >= 0 else -1)
        within = mask & (distance <= arc)
        phi = distance[within] / radius
        result[within] = along[within] + np.outer(radius * np.sin(phi), panel_direction) + np.outer(
            radius * (1 - np.cos(phi)), perpendicular)
        beyond = mask & (distance > arc)
        before = along[beyond] + panel_direction * arc
        after = along[beyond] + panel_direction * radius * math.sin(abs(angle)) + perpendicular * radius * (1 - math.cos(angle))
        result[beyond] = _rotate(points[beyond] - before, axis, angle) + after
    return result


def _frames(steps):
    frames = []
    for step in steps:
        original = np.array([step["start"], step["end"], step["start"] + step["side"] * step["moving_sign"] * 2])
        points = original.copy()
        for prior, frame in zip(steps, frames):
            points = _apply(points, original, prior, frame)
        frames.append(points)
    return frames


def _deform(triangles, steps, frames):
    original = triangles.reshape(-1, 3).astype(np.float64)
    flat = original.copy()
    flat[:, 1] = 0
    samples, inverse = np.unique(flat, axis=0, return_inverse=True)
    def transform(source):
        result = source.copy()
        for step, frame in zip(steps, frames):
            result = _apply(result, source, step, frame)
        return result
    epsilon = 1e-4
    dx, dz = np.array([epsilon, 0, 0]), np.array([0, 0, epsilon])
    tangent_x = transform(samples + dx) - transform(samples - dx)
    tangent_z = transform(samples + dz) - transform(samples - dz)
    normals = np.cross(tangent_z, tangent_x)
    lengths = np.linalg.norm(normals, axis=1)
    if np.any(lengths < 1e-12):
        raise ValueError("The formed sheet has an unresolved surface normal.")
    moved = transform(samples)[inverse] + original[:, 1, None] * (normals / lengths[:, None])[inverse]
    return _mesh(moved.reshape(-1, 3, 3))


def replay_bending_material(source, plan, *, units="mm"):
    """Preserve actual incoming triangles and retain one result per explicit bend.

    Coordinates use XZ for the flat sheet and Y for thickness. Step selection,
    order, angles, and tooling remain the caller's manufacturing authority.
    """
    if units != "mm":
        raise ValueError("Bending material replay requires millimetre units.")
    dimensions, steps = _steps(deepcopy(plan))
    if not isinstance(source, trimesh.Trimesh) or not source.faces.size or len(source.faces) > MAX_TRIANGLES:
        raise ValueError("Bending requires the actual incoming material triangle mesh.")
    triangles = np.asarray(source.triangles, dtype=np.float32).copy()
    if not np.isfinite(triangles).all():
        raise ValueError("The incoming material contains invalid coordinates.")
    bounds = np.array([triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))])
    length, width, thickness = dimensions
    if (abs(bounds[0, 1] + thickness / 2) > 0.002 or abs(bounds[1, 1] - thickness / 2) > 0.002
        or bounds[0, 0] < -length / 2 - 0.002 or bounds[1, 0] > length / 2 + 0.002
        or bounds[0, 2] < -width / 2 - 0.002 or bounds[1, 2] > width / 2 + 0.002):
        raise ValueError("The incoming material does not match the flat frame and sheet thickness.")
    before = _mesh(triangles)
    if not before.is_watertight or not before.is_winding_consistent or before.volume <= 0:
        raise ValueError("The incoming material must contain a closed, consistently oriented sheet surface.")
    if before.body_count != 1:
        raise ValueError("Bending requires one connected material piece for this process occurrence.")
    for step in steps:
        distance = (before.vertices - step["start"]) @ step["side"] * step["moving_sign"]
        moving_arc = step["radius"] * abs(step["angle"]) * (1 if step["fixed"] else step["ratio"])
        fixed_arc = 0 if step["fixed"] else step["radius"] * abs(step["angle"]) * (1 - step["ratio"])
        if distance.min() >= -fixed_arc - 0.001 or distance.max() <= moving_arc + 0.001:
            raise ValueError(f"Bending step {step['id']} has insufficient material on its fixed or moving panel.")
    initial_volume = float(before.volume)
    precision = deepcopy(before.metadata["stl_precision"])
    subdivided = _subdivide(before.triangles, steps)
    frames, passes = _frames(steps), []
    for index, step in enumerate(steps):
        after = _deform(subdivided, steps[:index + 1], frames[:index + 1])
        if not after.is_watertight or not after.is_winding_consistent or after.volume <= 0:
            raise ValueError(f"Bending step {step['id']} has no valid closed material surface.")
        if after.body_count != 1 or after.euler_number != before.euler_number:
            raise ValueError(f"Bending step {step['id']} changed the material surface topology.")
        volume_change = abs(float(after.volume) - initial_volume) / initial_volume
        if volume_change > 0.01:
            raise ValueError(f"Bending step {step['id']} exceeds the kinematic material-volume limit.")
        report = {"model_revision": MODEL_REVISION, "geometry_source": "actual_incoming_material",
            "volume_mm3": float(after.volume), "initial_volume_mm3": initial_volume,
            "relative_volume_change": volume_change, "fidelity": "kinematic_mesh",
            "input_stl_precision": precision, "output_stl_precision": after.metadata["stl_precision"],
            "validation_status": "not_verified", "manufacturing_ready": False,
            "reasons": list(LIMITATIONS)}
        passes.append(BendingMaterialPass(step["id"], before, after, report))
        before = after
    return BendingMaterialReplay(before, passes, {"model_revision": MODEL_REVISION,
        "geometry_source": "actual_incoming_material", "manufacturing_ready": False,
        "reasons": list(LIMITATIONS), "operation_count": len(passes)})
