"""Where the incoming material of a feature starts and ends along the tool axis.

A feature cut (a hole, a circular-milled bore, a reamed hole) goes into the material that is there when the
operation starts. The tool moves through the air above that material, so its flutes cut only the material height
and its reach is the depth from the setup face to the deepest cut (``tool_selection.tool_reach``):

    entry = setup face - top of the material inside the feature footprint
    far   = setup face - bottom of the material inside the footprint

A stepped part shows why: a hole in a 50 mm section 100 mm below the setup face needs a cutter that reaches
150 mm, but cuts only 50 mm of material once the waste above the section is gone. A through feature goes to the
bottom of the material column, not to the bottom of the stock box.
"""
from __future__ import annotations

# The probe is a little smaller than the feature: a coaxial cut of the same size made before leaves no sliver.
PROBE_MARGIN_MM = 0.005


def material_span(shape, face_selector: str, *, cx: float, cy: float, radius: float) -> tuple[float, float] | None:
    """(entry, far) depths below the setup face of the incoming material inside the circle at (cx, cy), in the
    setup frame of ``face_selector``; None when the circle has no material."""
    import cadquery as cq
    from .milling_replay import _boolean, finite
    from .profile_milling import _setup_plane

    radius = finite(radius, "feature radius", positive=True)
    plane, mirror = _setup_plane(face_selector)
    local = plane.toLocalCoords(shape.val())
    bounds = local.BoundingBox()
    probe = cq.Solid.makeCylinder(max(radius - PROBE_MARGIN_MM, radius * 0.5), bounds.zlen + 2.0,
                                  cq.Vector(finite(cx, "feature X position"), finite(cy, "feature Y position") * mirror,
                                            bounds.zmin - 1.0))
    try:
        common = _boolean(local, [probe], "common")
    except ValueError:
        # No Boolean on this material (a ledger layer solid can refuse it): the cut keeps the stock box rule.
        return None
    if not common.isValid() or not common.Solids() or common.Volume() <= 1e-6:
        return None
    # A box of curved faces can be a little larger than the shape: the column stays inside the material bounds.
    column = common.BoundingBox()
    top, bottom = min(column.zmax, bounds.zmax), max(column.zmin, bounds.zmin)
    return bounds.zmax - top, bounds.zmax - bottom


def feature_depths(shape, face_selector: str, *, cx: float, cy: float, radius: float, depth: float,
                   through: bool) -> tuple[float, float]:
    """(start_depth, depth) of a feature cut. A through feature ends at the bottom of its material column; a
    blind feature keeps its depth from the setup face. The start is the top of the material (0 without material
    above the floor, so the cut keeps its old rule and its own checks)."""
    span = material_span(shape, face_selector, cx=cx, cy=cy, radius=radius)
    if span is None:
        return 0.0, depth
    entry, far = span
    if through:
        return entry, far
    return (entry if entry < depth - 1e-6 else 0.0), depth
