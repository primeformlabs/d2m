"""The material of a milling program as one solid less 2D cuts (docs/MILLING_LAYER_REPLAY.md, "Material ledger").

A 3-axis cut removes a 2D region of its setup frame between two heights of that frame. The ledger keeps

    material = base − cut 1 − cut 2 − ...

where ``base`` is the first solid of the chain (most of the time the stock block) and each cut is (setup frame,
low height, high height, 2D region). The layers of the next operation come from this record: the plane section of
the base mesh less the sections of the cuts, all 2D polygon math. They do not come from the B-Rep result of the
operation before, so one damaged B-Rep state cannot change the layers of the operations after it.

The setup frames are the six cardinal frames. A cut of another frame is a 2D region in the query frame too:

* the same tool axis (for example >Z and <Z): the region with the axes turned, between the two mapped heights;
* a crossing tool axis (for example >Z and >X): at the height h the section is the set of rectangles
  (the chords of the region on the line of h) × (the height range of the cut).

The ledger is attached to the CadQuery workplane of the operation result (``attach``). An operation that does not
record its cuts returns a new workplane without a ledger; the next layer operation starts a new ledger from its
B-Rep (``MaterialLedger.of``).
"""
from __future__ import annotations

import math

import numpy as np

from . import layers as L

ATTRIBUTE = "_subcad_material_ledger"
# Largest number of cuts that one ledger keeps; above it the next operation starts from its B-Rep (bounded memory).
MAX_CUTS = 2000


def plane_frame(plane):
    """(rotation, origin) of a CadQuery plane: rotation columns are the setup x, y, z in part coordinates."""
    rotation = np.array([plane.xDir.toTuple(), plane.yDir.toTuple(), plane.zDir.toTuple()], float).T
    exact = np.rint(rotation)
    if np.abs(rotation - exact).max() > 1e-9 or np.abs(exact).sum() != 3:
        raise ValueError("The material ledger requires a cardinal setup frame.")
    return exact.astype(np.int64), np.array(plane.origin.toTuple(), float)


def _key(frame):
    rotation, origin = frame
    return tuple(rotation.reshape(-1).tolist()), tuple(np.round(origin, 9).tolist())


class Cut:
    """A 2D region of one setup frame (integer paths, layers.SCALE) between two heights of that frame."""

    __slots__ = ("frame", "z_low", "z_high", "region")

    def __init__(self, frame, z_low: float, z_high: float, region):
        self.frame = frame
        self.z_low, self.z_high = float(z_low), float(z_high)
        self.region = region


def disk(x: float, y: float, radius: float):
    """The region of a cutter disk (chords within the arc tolerance)."""
    return L.disk_sweep([(x, y)], radius)


def drill_cuts(frame, x: float, y: float, z_tip: float, radius: float, tip_radius: float, point_height: float,
               flute_length: float) -> list[Cut]:
    """The cuts of one axial plunge with the tip at ``z_tip``: the point as bands of ``SLOPED_LAYER_MM`` (each one
    with the radius at its middle) and the full diameter up to the flute top."""
    cuts = []
    if point_height > 0:
        count = max(1, math.ceil(point_height / L.SLOPED_LAYER_MM - 1e-9))
        for index in range(count):
            a, b = point_height * index / count, point_height * (index + 1) / count
            middle = tip_radius + (radius - tip_radius) * (a + b) / (2 * point_height)
            if middle > 0:
                cuts.append(Cut(frame, z_tip + a, z_tip + b, disk(x, y, middle)))
    if flute_length > point_height:
        cuts.append(Cut(frame, z_tip + point_height, z_tip + flute_length, disk(x, y, radius)))
    return cuts


class _Mapped:
    """A cut in the query frame: same axis (region and heights) or crossing axis (data for the line chords)."""

    __slots__ = ("same", "z_low", "z_high", "region", "cut", "transform", "shift", "limits", "sloped")

    def __init__(self, cut: Cut, query):
        rotation, origin = query
        transform = rotation.T @ cut.frame[0]
        shift = rotation.T @ (cut.frame[1] - origin)
        self.cut, self.transform, self.shift = cut, transform, shift
        self.same = abs(transform[2, 2]) == 1
        if self.same:
            heights = (transform[2, 2] * cut.z_low + shift[2], transform[2, 2] * cut.z_high + shift[2])
            self.z_low, self.z_high = min(heights), max(heights)
            block = transform[:2, :2]
            offset = np.rint(shift[:2] * L.SCALE).astype(np.int64)
            mirrored = round(float(np.linalg.det(block))) < 0
            region = []
            for path in cut.region:
                points = np.asarray(path, np.int64) @ block.T + offset
                values = [tuple(point) for point in points.tolist()]
                region.append(values[::-1] if mirrored else values)
            self.region = region
            self.limits, self.sloped = (self.z_low, self.z_high), ()
        else:
            # The query height is ±(one in-plane coordinate of the cut): the region spans its range on that axis.
            axis = int(np.nonzero(transform[2, :2])[0][0])
            sign, other = transform[2, axis], 1 - axis
            heights = lambda values: sign * np.asarray(values, float) / L.SCALE + shift[2]
            limits, sloped = [], []
            for path in cut.region:
                points = np.asarray(path, float)
                following = np.roll(points, -1, axis=0)
                level = points[:, axis] != following[:, axis]
                slanted = level & (points[:, other] != following[:, other])
                # An edge along the query axis keeps its chord end: a layer limit at its two ends. A slanted edge
                # moves the chord end with the height: the thinner layers over its range.
                limits.extend(heights(points[~slanted, axis]).tolist())
                limits.extend(heights(following[~slanted, axis]).tolist())
                for a, b in zip(heights(points[slanted, axis]), heights(following[slanted, axis])):
                    sloped.append((min(a, b), max(a, b)))
            values = heights([point[axis] for path in cut.region for point in path])
            self.z_low, self.z_high = float(values.min()), float(values.max())
            self.limits, self.sloped = limits, _merged_ranges(sloped)
            self.region = None

    def section(self, height: float):
        """The cut region in the plane of the query height (crossing axis)."""
        transform, shift, cut = self.transform, self.shift, self.cut
        axis = int(np.nonzero(transform[2, :2])[0][0])
        other = 1 - axis
        level = (height - shift[2]) / transform[2, axis] * L.SCALE
        chords = _chords(cut.region, axis, other, level)
        rectangles = []
        for low, high in chords:
            corners = []
            for along in (low / L.SCALE, high / L.SCALE):
                for z in (cut.z_low, cut.z_high):
                    local = np.zeros(3)
                    local[axis], local[other], local[2] = level / L.SCALE, along, z
                    corners.append(transform @ local + shift)
            corners = np.array(corners)
            x0, y0 = corners[:, 0].min(), corners[:, 1].min()
            x1, y1 = corners[:, 0].max(), corners[:, 1].max()
            if x1 - x0 > 1e-9 and y1 - y0 > 1e-9:
                rectangles.append([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
        return L.to_paths(rectangles)


def _merged_ranges(ranges):
    merged = []
    for low, high in sorted(ranges):
        if merged and low <= merged[-1][1] + L.Z_MERGE_MM:
            merged[-1][1] = max(merged[-1][1], high)
        else:
            merged.append([low, high])
    return [tuple(item) for item in merged]


def _chords(region, axis: int, other: int, level: float):
    """The intervals of the line {coordinate ``axis`` = level} inside the region (even-odd, half-open crossings)."""
    crossings = []
    for path in region:
        points = np.asarray(path, float)
        first, second = points, np.roll(points, -1, axis=0)
        a, b = first[:, axis], second[:, axis]
        crossed = (a <= level) != (b <= level)
        if not np.any(crossed):
            continue
        t = (level - a[crossed]) / (b[crossed] - a[crossed])
        crossings.extend((first[crossed, other] + t * (second[crossed, other] - first[crossed, other])).tolist())
    crossings.sort()
    return [(crossings[index], crossings[index + 1]) for index in range(0, len(crossings) - 1, 2)
            if crossings[index + 1] > crossings[index]]


class MaterialLedger:
    """The base solid (part frame) and the cuts made from it since."""

    def __init__(self, base, cuts: tuple = (), meshes: dict | None = None):
        self.base = base
        self.cuts = tuple(cuts)
        self._meshes = {} if meshes is None else meshes  # shared by the ledgers of one base
        self.brep_from_layers = False  # the B-Rep of the attached workplane is ``solid()`` of this ledger

    @classmethod
    def of(cls, workplane) -> "MaterialLedger":
        """The ledger attached to the workplane, or a new one with its solid as the base."""
        ledger = getattr(workplane, ATTRIBUTE, None)
        if isinstance(ledger, cls):
            return ledger
        return cls(workplane.val())

    @staticmethod
    def layer_brep(workplane) -> bool:
        """True when the workplane's B-Rep is the layer solid of its ledger (``solid``): a Boolean on it is slow and
        gives nothing that the ledger does not."""
        ledger = getattr(workplane, ATTRIBUTE, None)
        return isinstance(ledger, MaterialLedger) and ledger.brep_from_layers

    def attach(self, workplane):
        if len(self.cuts) <= MAX_CUTS:
            setattr(workplane, ATTRIBUTE, self)
        return workplane

    def with_cuts(self, cuts) -> "MaterialLedger":
        return MaterialLedger(self.base, self.cuts + tuple(cuts), self._meshes)

    def mesh(self, frame) -> L.SolidMesh:
        """The base mesh in the query frame (one mesh of the base; the frames turn its vertices)."""
        key = _key(frame)
        if key not in self._meshes:
            part = self._meshes.get("part") or L.SolidMesh.of(self.base)
            self._meshes["part"] = part
            rotation, origin = frame
            self._meshes[key] = L.SolidMesh((part.vertices - origin) @ rotation, part.triangles)
        return self._meshes[key]

    def layered(self, frame, z_low: float, z_high: float, levels=()) -> L.LayeredMaterial:
        """The material between two heights of the query frame as layers (base sections less the cut sections)."""
        mesh = self.mesh(frame)
        mapped = [_Mapped(cut, frame) for cut in self.cuts]
        mapped = [item for item in mapped if item.z_high > z_low + L.Z_MERGE_MM and item.z_low < z_high - L.Z_MERGE_MM]
        same = [item for item in mapped if item.same]
        crossing = [item for item in mapped if not item.same]
        limits = [value for item in mapped for value in item.limits]
        sloped = [value for item in crossing for value in item.sloped]
        unions: dict[tuple, list] = {}

        def section(height: float, base_paths):
            covering = tuple(index for index, item in enumerate(same)
                             if item.z_low - L.Z_MERGE_MM <= height <= item.z_high + L.Z_MERGE_MM)
            if covering not in unions:
                unions[covering] = L.union([path for index in covering for path in same[index].region])
            removed = unions[covering]
            extra = [path for item in crossing if item.z_low < height < item.z_high for path in item.section(height)]
            if extra:
                removed = L.union(removed, extra)
            return L.difference(base_paths, removed) if removed else base_paths

        return L.LayeredMaterial.of_solid(None, z_low, z_high, levels, mesh=mesh, limits=limits, sloped=sloped,
                                          section=section)

    def volume_in(self, frame) -> tuple[float, L.LayeredMaterial]:
        """The material volume (layers over the full height of the base in the frame) and those layers."""
        mesh = self.mesh(frame)
        heights = mesh.vertices[:, 2]
        material = self.layered(frame, float(heights.min()), float(heights.max()))
        return material.volume(), material

    def solid(self, plane):
        """A B-Rep of the ledger material in the plane's local frame, made from its layers with no Boolean: one prism
        for each face of each run of equal layers. The outlines have the player tolerance (``DISPLAY_TOLERANCE_MM``);
        the checks use the layers, not this B-Rep. A plane section of this solid is a layer outline."""
        import cadquery as cq
        _, material = self.volume_in(plane_frame(plane))
        return _layer_solid(material, cq)


def _layer_solid(material: L.LayeredMaterial, cq):
    """The layer material as prisms: one for each face of each run of layers with equal polygons (no Boolean)."""
    import pyclipper
    from .layer_player import DISPLAY_TOLERANCE_MM

    def outline(paths):
        return [path for path in pyclipper.CleanPolygons(paths, DISPLAY_TOLERANCE_MM * L.SCALE) if len(path) >= 3]
    solids, run = [], None
    for layer in material.layers:
        if run is not None and run[2] == layer.material and abs(run[1] - layer.z_low) <= L.Z_MERGE_MM:
            run[1] = layer.z_high
            continue
        if run is not None and run[2]:
            solids.extend(L.prisms(outline(run[2]), run[0], run[1]))
        run = [layer.z_low, layer.z_high, layer.material]
    if run is not None and run[2]:
        solids.extend(L.prisms(outline(run[2]), run[0], run[1]))
    if not solids:
        raise ValueError("The material ledger holds no material.")
    return solids[0] if len(solids) == 1 else cq.Compound.makeCompound(solids)
