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


class LocalReach:
    """The reach that a tool needs for a round feature (Rajesh, 2026-10-26): from the highest material inside the
    region that the tool and its holder take above the feature (the cutter path grown by the holder radius and the
    clearance), down to the deepest cut. Taller material farther away has no effect. The material is the incoming
    material of the operation (after level roughing). ``path``: "axial" (drill, reamer: the path is the axis) or
    "circle" (circular milling: the path is the hole circle less the cutter radius)."""

    def __init__(self, shape, face_selector: str, *, cx: float, cy: float, hole_radius: float, depth: float,
                 entry: float, path: str = "axial"):
        self.shape, self.face, self.cx, self.cy = shape, face_selector, cx, cy
        self.hole_radius, self.depth, self.entry, self.path = hole_radius, depth, entry, path
        self._tops: dict[float, float] = {}

    def envelope_radius(self, tool) -> float:
        from .tool_selection import HOLDER_CLEARANCE_MM, holder_radius
        if self.path == "circle":
            return max(self.hole_radius - tool.diameter / 2, 0.0) + holder_radius(tool) + HOLDER_CLEARANCE_MM
        return max(self.hole_radius, holder_radius(tool)) + HOLDER_CLEARANCE_MM

    def top(self, radius: float) -> float:
        """The depth below the setup face of the highest material within the radius (at most the feature entry)."""
        key = round(radius, 3)
        if key not in self._tops:
            span = material_span(self.shape, self.face, cx=self.cx, cy=self.cy, radius=radius)
            self._tops[key] = min(span[0], self.entry) if span else self.entry
        return self._tops[key]

    def __call__(self, tool) -> float:
        return max(self.depth - self.top(self.envelope_radius(tool)), 0.0)
