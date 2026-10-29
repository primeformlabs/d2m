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


# A pocket wall is the material just outside the pocket outline: its top is where the flutes start to cut a wall.
WALL_BAND_MM = 1.0
# Material in the band counts from this area: 2 mm of wall in the 1 mm band. The rounded joins of the layer outlines
# leave slivers of less than 1 mm² at the corners of a kept outline; they are not a wall.
BAND_MIN_AREA_MM2 = 2.0


def _offset_rings(points, distance: float) -> list[list[tuple[float, float]]]:
    """The outline grown by ``distance`` (round corners), as closed rings."""
    import pyclipper
    scale = 1000.0
    offset = pyclipper.PyclipperOffset()
    offset.AddPath([(round(x * scale), round(y * scale)) for x, y in points], pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
    return [[(x / scale, y / scale) for x, y in ring] for ring in offset.Execute(distance * scale) if len(ring) >= 3]


def _ring_area(ring) -> float:
    return abs(sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(ring, ring[1:] + ring[:1]))) / 2


def band_span(shape, face_selector: str, points, *, width: float) -> tuple[float, float] | None:
    """(entry, far) depths below the setup face of the incoming material in the band ``width`` wide outside the closed
    outline ``points`` (setup frame); None when the band has no material."""
    import cadquery as cq
    from .milling_replay import _boolean, finite
    from .profile_milling import _setup_plane

    width = finite(width, "band width", positive=True)
    plane, mirror = _setup_plane(face_selector)
    local = plane.toLocalCoords(shape.val())
    bounds = local.BoundingBox()
    outline = [(float(x), float(y) * mirror) for x, y in points]
    rings = _offset_rings(outline, width)
    if len(outline) < 3 or not rings:
        return None
    outer = max(rings, key=_ring_area)
    ledgered = _ledger_band_span(shape, plane, bounds, outer, outline)
    if ledgered is not False:
        return ledgered
    base = bounds.zmin - 1.0

    def wire(ring):
        return cq.Wire.makePolygon([cq.Vector(x, y, base) for x, y in ring], close=True)
    try:
        face = cq.Face.makeFromWires(wire(outer), [wire(list(reversed(outline)))])
        probe = cq.Solid.extrudeLinear(face, cq.Vector(0, 0, bounds.zlen + 2.0))
        common = _boolean(local, [probe], "common")
    except (ValueError, RuntimeError):
        return None
    if not common.isValid() or not common.Solids() or common.Volume() <= 1e-6:
        return None
    column = common.BoundingBox()
    top, bottom = min(column.zmax, bounds.zmax), max(column.zmin, bounds.zmin)
    return bounds.zmax - top, bounds.zmax - bottom


def _ledger_band_span(shape, plane, bounds, outer, outline):
    """``band_span`` from the material ledger of the workplane (``material_ledger.py``): the layers of the setup frame
    that meet the band. After level roughing the B-Rep is the layer solid of the ledger, and a Boolean on it can fail
    or be slow. False: the workplane has no ledger (the Boolean probe decides)."""
    from . import layers as L
    from .material_ledger import ATTRIBUTE, MaterialLedger, plane_frame
    ledger = getattr(shape, ATTRIBUTE, None)
    if not isinstance(ledger, MaterialLedger):
        return False
    try:
        material = ledger.layered(plane_frame(plane), bounds.zmin, bounds.zmax)
    except ValueError:
        return False
    band = L.difference(L.to_paths([outer]), L.to_paths([outline]))
    hits = [layer for layer in material.layers if L.area(L.intersection(layer.material, band)) >= BAND_MIN_AREA_MM2]
    if not hits:
        return None
    top, bottom = max(layer.z_high for layer in hits), min(layer.z_low for layer in hits)
    return bounds.zmax - min(top, bounds.zmax), bounds.zmax - max(bottom, bounds.zmin)


def pocket_wall_start(shape, face_selector: str, points, depth: float) -> float | None:
    """Where the wall of a pocket starts below the setup face: the top of the material just outside its outline
    (``WALL_BAND_MM``). Above it the cutter cuts material with nothing beside it (the waste over a part level that
    level roughing kept in the pocket column), so the flutes cut only the wall below it. None: no material beside
    the outline, or no geometry."""
    span = band_span(shape, face_selector, points, width=WALL_BAND_MM)
    return None if span is None else min(span[0], depth)


class PocketLocalReach:
    """The reach that a tool needs for a pocket (Rajesh, 2026-10-26, as ``LocalReach``): from the highest material in
    the band that its holder takes beside the outline (the holder radius beyond the cutter radius and the clearance)
    down to the pocket floor. The material inside the outline is cut on the way down, so it does not count."""

    def __init__(self, shape, face_selector: str, points, *, depth: float, wall: float):
        self.shape, self.face, self.points, self.depth, self.wall = shape, face_selector, points, depth, wall
        self._tops: dict[float, float] = {}

    def band(self, tool) -> float:
        from .tool_selection import HOLDER_CLEARANCE_MM, holder_radius
        return max(holder_radius(tool) - tool.diameter / 2, 0.0) + HOLDER_CLEARANCE_MM

    def top(self, width: float) -> float:
        key = round(width, 3)
        if key not in self._tops:
            span = band_span(self.shape, self.face, self.points, width=width)
            self._tops[key] = min(span[0], self.wall) if span else self.wall
        return self._tops[key]

    def __call__(self, tool) -> float:
        return max(self.depth - self.top(self.band(tool)), 0.0)
