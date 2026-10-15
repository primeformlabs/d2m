"""Native nominal-solid comparison. This does not qualify manufacturing tolerances."""
from dataclasses import dataclass
import math
from pathlib import Path

from .milling_replay import _boolean, finite


@dataclass
class NominalComparison:
    report: dict
    target: object
    missing_material: object | None
    remaining_stock: object | None


def _solid(source, name):
    import cadquery as cq
    source = getattr(source, "native", source)
    source = getattr(source, "_shape", source)
    source = source.val() if hasattr(source, "val") else source
    if not isinstance(source, cq.Shape) or not source.isValid():
        raise ValueError(f"The {name} does not contain valid native geometry.")
    solids = source.Solids()
    if len(solids) != 1:
        raise ValueError(f"The {name} requires one closed part solid. It contains {len(solids)} solids.")
    solid = solids[0]
    finite(solid.Volume(), f"{name} volume", positive=True)
    return solid


def load_nominal_step(path):
    """Load exact STEP geometry in millimetres without a mesh or bounding-box fallback."""
    import cadquery as cq
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    path = Path(path)
    if path.suffix.lower() not in {".step", ".stp"} or not path.is_file():
        raise ValueError("The required part needs an existing STEP file.")
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise ValueError("The required part STEP file cannot be read by the native engine.")
    if not hasattr(reader, "SetSystemLengthUnit"):
        raise ValueError("The native STEP reader cannot establish millimetre units.")
    reader.SetSystemLengthUnit(1.0)
    if reader.TransferRoots() <= 0 or reader.OneShape().IsNull():
        raise ValueError("The required part STEP file contains no transferable solid.")
    return _solid(cq.Shape.cast(reader.OneShape()), "required part")


def _matrix(values):
    import cadquery as cq
    from OCP.gp import gp_Trsf
    if not isinstance(values, (list, tuple)) or len(values) != 16:
        raise ValueError("The target comparison requires an explicit sixteen-value datum transform.")
    values = [finite(value, "target datum transform") for value in values]
    if any(abs(values[index] - expected) > 1e-9 for index, expected in ((12, 0), (13, 0), (14, 0), (15, 1))):
        raise ValueError("The target datum transform has an invalid homogeneous row.")
    rows = [values[start:start + 3] for start in (0, 4, 8)]
    if any(abs(sum(a * b for a, b in zip(first, second)) - (1 if i == j else 0)) > 1e-8
            for i, first in enumerate(rows) for j, second in enumerate(rows)):
        raise ValueError("The target datum transform must preserve physical dimensions.")
    a, b, c = rows
    determinant = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])
    if abs(determinant - 1) > 1e-8:
        raise ValueError("The target datum transform must be a proper rigid transform.")
    transform = gp_Trsf()
    transform.SetValues(*values[:12])
    return cq.Matrix(transform)


def _bounds(shape):
    box = shape.BoundingBox()
    return {"x_min": box.xmin, "x_max": box.xmax, "y_min": box.ymin, "y_max": box.ymax,
        "z_min": box.zmin, "z_max": box.zmax}


def compare_nominal_solids(candidate, target, *, target_to_part_matrix):
    """Compare actual solids in one declared part frame. Keep signed difference solids."""
    candidate = _solid(candidate, "simulated result")
    target = load_nominal_step(target) if isinstance(target, (str, Path)) else _solid(target, "required part")
    target = _solid(target.transformShape(_matrix(target_to_part_matrix)), "transformed required part")
    missing = _boolean(target, [candidate], "cut")
    remaining = _boolean(candidate, [target], "cut")
    if not missing.isValid() or not remaining.isValid():
        raise ValueError("The native target difference has invalid geometry. The required shape is not verified.")
    missing_volume = finite(missing.Volume(), "missing material volume")
    remaining_volume = finite(remaining.Volume(), "remaining stock volume")
    if missing_volume < 0 or remaining_volume < 0:
        raise ValueError("The native target difference has a negative volume.")
    target_volume, candidate_volume = target.Volume(), candidate.Volume()
    numerical_volume_limit = max(1e-7, max(target_volume, candidate_volume) * 1e-12)
    difference = missing_volume > numerical_volume_limit or remaining_volume > numerical_volume_limit
    report = {"schema_version": "nominal-solid-comparison/v1", "method": "native_brep_symmetric_difference",
        "status": "differences" if difference else "no_resolved_difference", "units": "mm",
        "coordinate_system": "part_mm", "target_to_part_matrix": list(target_to_part_matrix),
        "target_volume_mm3": target_volume, "candidate_volume_mm3": candidate_volume,
        "missing_material_mm3": missing_volume, "remaining_stock_mm3": remaining_volume,
        "numerical_volume_limit_mm3": numerical_volume_limit,
        "target_bounds": _bounds(target), "candidate_bounds": _bounds(candidate),
        "tolerance_verified": False, "manufacturing_ready": False,
        "reasons": ["The comparison uses the required nominal STEP solid. It does not qualify the requested manufacturing tolerances.",
            "Tool, fixture, datum, runout, inspection, and process qualification remain required."]}
    return NominalComparison(report, target, missing if missing_volume > numerical_volume_limit else None,
        remaining if remaining_volume > numerical_volume_limit else None)
