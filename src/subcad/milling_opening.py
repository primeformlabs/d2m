"""Milling verdict by the morphological opening, one setup and one 2D layer at a time.

A 3-axis setup has one fixed tool axis (+Z of the setup frame). For each layer of the setup:

    R      = stock section - part section              (the material to remove)
    shadow = the union of the part sections at and above the layer
    left   = R minus the region that the cutter can reach

A flat end mill of radius r reaches exactly the morphological opening of the free region (the plane
less the shadow): the union of all its disks that do not touch the part. Its complement inside R is
R intersected with the closing of the shadow:

    left = R ∩ offset(offset(shadow, +r), -r)

The material left is therefore the inside corners (radius r by construction), the slots narrower than
2r and the undercuts. No pass, no toolpath and no 3D Boolean is used.

A cutter with a profile (ball end mill, bull nose, drill point) has the radius rho(h) at the height h
above its tip. With its tip at height t, its centre must stay out of

    obstacle(t) = union over the layers above t of offset(part layer, rho(layer top - t))

and at the height z it reaches the union over the tips t in [z - c, z] of offset(free centres(t), rho(z - t)),
where c is the profile height (ball: r; bull nose: corner radius; drill: point length). A tip lower than
z - c gives no more reach (its obstacle is larger, its radius is the same). So

    left(z) = R ∩ intersection over t of offset(obstacle(t), -rho(z - t))

Reach: a layer deeper below the setup face than min(flute length, stick-out, reach) is out of reach for
that cutter. The setup face is the part top in the setup frame: the plan faces it first (D2O "Face and
square first datums"), and the old replay measures the cutting depth from the incoming material top. A tip does not go below the stock bottom (the table and the fixture are there).

Layers: the limits are the heights of the planar (horizontal) faces of the stock and the part. A layer
that a sloped or curved face crosses is divided so that the face section moves by at most ``CHORD_MM``
inside one layer (chord bound). The material of a layer is its middle section; the shadow takes the
sections at the bottom, the middle and the top of each layer.

All 2D work uses ``layers.py`` (pyclipper, 1 unit = 1 nm, round joins, arc tolerance 0.25 µm). A region
thinner than the contact tolerance (1 µm) does not count, as in the layer replay.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Iterable, Sequence

import numpy as np

from . import layers as L

# Largest horizontal movement of a sloped or curved face section inside one layer (mm).
CHORD_MM = 0.01
# Thinnest layer of a sloped interval (mm), as the 0.1 mm sloped layers of the layer replay (PR #12). The volume of
# a layer comes from its middle section, and the middle-section rule is exact to the second order: an almost
# horizontal face (the top of a horizontal cylinder) needs no thinner layers for its volume.
MIN_LAYER_MM = 0.1
# Largest number of layers of one setup; above it the chord bound grows, up to MAX_CHORD_MM (then the layer
# count grows instead: a side setup across round holes needs about 2 R / chord layers for each hole).
MAX_LAYERS = 400
MAX_CHORD_MM = 0.05
# A cutter centre fits where the gap is this much smaller than the cutter (mm): the mesh chords (1 µm) and the
# polygon chords of the obstacle offsets (1 µm) lie inside the true surfaces, so a hole of the cutter size fits.
FIT_MM = 0.002
# Arc tolerance of the obstacle offsets (mm). Its chords lie inside the true offset; ``FIT_MM`` covers them.
OBSTACLE_ARC_MM = 0.001
# A drill makes a hole up to this fraction of its radius larger than itself (a spot of free centres).
DRILL_HOLE_RATIO = 0.1
# Height samples in the profile height of a cutter with a profile (ball, bull nose, drill point).
PROFILE_SAMPLES = 8
# The setup frames of the six cardinal faces (D2O ``milling_setup_frame.FACE_AXES``): profile axes u, v and the normal.
FACE_AXES = {
    ">Z": ("x", "y", "z", 1), "<Z": ("x", "y", "z", -1),
    ">X": ("y", "z", "x", 1), "<X": ("y", "z", "x", -1),
    ">Y": ("x", "z", "y", 1), "<Y": ("x", "z", "y", -1),
}


# -- setups and cutters ----------------------------------------------------------------------------

@dataclass(frozen=True)
class Setup:
    """A fixed tool axis. ``matrix`` columns: the setup x, y and z directions in the part frame."""
    name: str
    matrix: tuple

    @classmethod
    def of(cls, value) -> "Setup":
        if isinstance(value, Setup):
            return value
        if isinstance(value, str):
            if value not in FACE_AXES:
                raise ValueError("A setup selector must be one of the six cardinal faces.")
            u, _, normal, sign = FACE_AXES[value]
            unit = lambda axis, scale: np.array([scale if key == axis else 0.0 for key in "xyz"])
            x_dir, z_dir = unit(u, 1.0), unit(normal, float(sign))
            y_dir = np.cross(z_dir, x_dir)
            return cls(value, tuple(map(tuple, np.column_stack([x_dir, y_dir, z_dir]))))
        name, matrix = value
        array = np.asarray(matrix, float)[:3, :3]
        if abs(np.linalg.det(array) - 1.0) > 1e-6:
            raise ValueError("A setup frame must be a rotation.")
        return cls(str(name), tuple(map(tuple, array)))

    @property
    def rotation(self) -> np.ndarray:
        return np.asarray(self.matrix, float)

    def to_setup(self, points: np.ndarray) -> np.ndarray:
        """Part-frame points in setup coordinates."""
        return points @ self.rotation

    def to_dict(self):
        return {"name": self.name, "setup_to_part": [list(row) for row in self.matrix]}


@dataclass(frozen=True)
class Cutter:
    """The rotational profile of a milling cutter, its reach and its shank."""
    kind: str
    diameter: float
    corner_radius: float = 0.0
    flute_length: float = 50.0
    stickout: float = 35.0
    reach: float = 0.0
    shank_diameter: float = 0.0
    tip_angle: float = 0.0
    name: str = ""

    @classmethod
    def of(cls, tool) -> "Cutter":
        """From a ``ToolSpec``, a ``ToolSpec.to_dict()`` row or a ``Cutter``."""
        if isinstance(tool, Cutter):
            return tool
        if isinstance(tool, dict):
            get = lambda *keys, default=0.0: next((float(tool[key]) for key in keys if tool.get(key) is not None), default)
            return cls(str(tool.get("tool_type") or "flat_endmill"), get("diameter_mm", "diameter"),
                       get("corner_radius_mm", "corner_radius"), get("flute_length_mm", "flute_length", default=50.0),
                       get("stickout_mm", default=35.0), get("reach_mm"), get("shank_diameter_mm", "shank_diameter"),
                       get("tip_angle_deg", "tip_angle"), str(tool.get("catalog_id") or tool.get("label") or ""))
        return cls(tool.tool_type, float(tool.diameter), float(tool.corner_radius or 0.0), float(tool.flute_length),
                   float(tool.stickout_mm), float(tool.reach_mm or 0.0), float(tool.shank_diameter or 0.0),
                   float(tool.tip_angle or 0.0), tool.catalog_id or "")

    @property
    def radius(self) -> float:
        return self.diameter / 2

    @property
    def shank_radius(self) -> float:
        return (self.shank_diameter or self.diameter) / 2

    @property
    def reach_depth(self) -> float:
        """The deepest layer below the setup face that the cutter cuts (``milling_replay.validate_cutter``)."""
        return min(self.flute_length, self.stickout, self.reach or self.flute_length)

    @property
    def profile_height(self) -> float:
        """The height above the tip where the cutter has its full radius."""
        r = self.radius
        if self.kind == "ball_endmill":
            return r
        if self.kind == "bull_nose" or (self.kind == "flat_endmill" and self.corner_radius > 0):
            return min(self.corner_radius, r)
        if self.kind == "drill" and 0 < self.tip_angle < 180:
            return r / math.tan(math.radians(self.tip_angle) / 2)
        return 0.0

    def radius_at(self, height: float) -> float:
        """The cutter radius at the height above its tip (the shank above the flutes)."""
        if height < 0:
            return 0.0
        if height > self.flute_length:
            return max(self.radius, self.shank_radius)
        r, c = self.radius, self.profile_height
        if height >= c or c <= 0:
            return r
        if self.kind == "drill":
            return height * r / c
        # Ball end mill and bull nose: a corner arc of radius c.
        return (r - c) + math.sqrt(max(c * c - (c - height) ** 2, 0.0))

    def to_dict(self):
        return {"kind": self.kind, "diameter_mm": self.diameter, "corner_radius_mm": self.corner_radius,
                "flute_length_mm": self.flute_length, "reach_depth_mm": self.reach_depth,
                "profile_height_mm": round(self.profile_height, 6), "name": self.name}


# -- layers of one setup ---------------------------------------------------------------------------

@dataclass
class SetupLayers:
    setup: Setup
    z_low: float
    z_high: float
    limits: list
    stock: list = field(default_factory=list)        # middle section of each layer
    part: list = field(default_factory=list)         # middle section of each layer
    part_band: list = field(default_factory=list)    # bottom ∪ middle ∪ top sections (shadow input)
    shadow: list = field(default_factory=list)       # union of part_band of this layer and all layers above
    chord_mm: float = CHORD_MM
    face: float = 0.0  # the setup face: the part top in the setup frame (the plan faces it first)
    planar: list = field(default_factory=list)  # the heights of the horizontal faces (floors)
    run: list = field(default_factory=list)     # the first layer of the run of equal part sections of each layer

    @property
    def count(self) -> int:
        return len(self.limits) - 1

    def heights(self, index):
        return self.limits[index], self.limits[index + 1]

    def layer_at(self, z: float) -> int:
        """The index of the layer that holds the height (the last layer for the top)."""
        import bisect
        return min(max(bisect.bisect_right(self.limits, z + L.Z_MERGE_MM) - 1, 0), self.count - 1)

    def material(self, index):
        return L.difference(self.stock[index], self.part[index])


def _section(mesh: L.SolidMesh, z: float, room: float) -> list:
    """The mesh section at the height; a plane through a mesh edge moves by a few nanometres inside the room."""
    for shift in (0.0, 1e-6, -1e-6, 3e-6, -3e-6):
        if abs(shift) * 2 < room or shift == 0.0:
            try:
                return mesh.section(z + shift)
            except ValueError:
                continue
    return mesh.section(z)


def rotated_mesh(mesh: L.SolidMesh, setup: Setup) -> L.SolidMesh:
    return L.SolidMesh(setup.to_setup(mesh.vertices), mesh.triangles)


def _horizontal_heights(mesh: L.SolidMesh) -> np.ndarray:
    flat = np.abs(mesh.normal_z) >= 1 - 1e-9
    return np.unique(np.round(mesh.vertices[mesh.triangles[flat]][..., 2].ravel(), 7))


def _slope_rates(mesh: L.SolidMesh, chord: float = CHORD_MM):
    """(low, high, rate) of each sloped triangle: the horizontal movement of its section per mm of height.

    A triangle whose section moves less than the chord bound over its whole height needs no division (the short
    triangles near the top of a horizontal cylinder)."""
    nz = np.abs(mesh.normal_z)
    sloped = (nz > 1e-9) & (nz < 1 - 1e-9) & (mesh.high - mesh.low > L.Z_MERGE_MM)
    rate = np.zeros(len(nz))
    rate[sloped] = nz[sloped] / np.sqrt(1 - nz[sloped] ** 2)
    sloped &= rate * (mesh.high - mesh.low) > chord
    return mesh.low[sloped], mesh.high[sloped], rate[sloped]


def _divide(first: float, second: float, low, high, rate, chord: float) -> list[float]:
    """The heights inside (first, second) so that no sloped section moves more than ``chord`` in one layer, with
    layers not thinner than ``MIN_LAYER_MM`` (scaled with the chord when the layer count is too large)."""
    thinnest = min(MIN_LAYER_MM * chord / CHORD_MM, 0.5)
    inside = (low < second - L.Z_MERGE_MM) & (high > first + L.Z_MERGE_MM)
    if not np.any(inside):
        return []
    low, high, rate = low[inside], high[inside], rate[inside]
    result, z = [], first
    while second - z > L.Z_MERGE_MM:
        step = second - z
        for _ in range(3):
            near = (low < z + step) & (high > z)
            if not np.any(near):
                break
            step = min(step, max(chord / float(rate[near].max()), thinnest))
        z = min(z + step, second)
        if second - z > L.Z_MERGE_MM:
            result.append(z)
        if len(result) > MAX_LAYERS * 20:
            break
    return result


def setup_layers(stock_mesh: L.SolidMesh, part_mesh: L.SolidMesh, setup, chord: float = CHORD_MM,
                 extra_meshes: Sequence = (), reach_depths: Sequence = (), divide_depth: float | None = None) -> SetupLayers:
    """The 2D layers of the stock and the part in the setup frame.

    ``extra_meshes``: more solids whose faces add layer limits (a comparison with another material state).
    ``reach_depths``: cutter reach depths below the setup face; each one adds a layer limit.
    ``divide_depth``: divide the sloped intervals only down to this depth below the setup face (the deepest cutter
    reach). Deeper, no cutter of the setup cuts, and the layers of the planar faces are enough."""
    setup = Setup.of(setup)
    stock, part = rotated_mesh(stock_mesh, setup), rotated_mesh(part_mesh, setup)
    extra = [rotated_mesh(mesh, setup) for mesh in extra_meshes]
    z_low, z_high = float(stock.vertices[:, 2].min()), float(stock.vertices[:, 2].max())
    face = min(float(part.vertices[:, 2].max()), z_high)
    planar = [z for z in np.concatenate([_horizontal_heights(mesh) for mesh in (stock, part, *extra)]) if z_low < z < z_high]
    planar += [face - depth for depth in reach_depths if z_low < face - depth < z_high]
    base = L._merged([z_low, z_high, *planar])
    rates = [np.concatenate(values) for values in zip(*[_slope_rates(mesh, chord) for mesh in (stock, part, *extra)])]
    while True:
        limits = [base[0]]
        cut = -math.inf if divide_depth is None else face - divide_depth
        for first, second in zip(base, base[1:]):
            if second > cut + L.Z_MERGE_MM:
                if first < cut - L.Z_MERGE_MM:
                    limits.append(cut)
                limits.extend(_divide(max(first, cut), second, *rates, chord))
            limits.append(second)
            if len(limits) > MAX_LAYERS * 4 and chord < MAX_CHORD_MM:
                break
        if len(limits) - 1 <= MAX_LAYERS or chord >= MAX_CHORD_MM:
            break
        chord = min(chord * 1.5, MAX_CHORD_MM)
    result = SetupLayers(setup, z_low, z_high, limits, chord_mm=chord, face=face, planar=base)
    for a, b in zip(limits, limits[1:]):
        middle = (a + b) / 2
        edge = min(1e-6, (b - a) / 4)
        result.stock.append(_section(stock, middle, b - a))
        mid = _section(part, middle, b - a)
        result.part.append(mid)
        result.part_band.append(L.union(mid, L.union(_section(part, a + edge, edge), _section(part, b - edge, edge))))
    for index, band in enumerate(result.part_band):
        same = index and band == result.part_band[index - 1]
        result.run.append(result.run[-1] if same else index)
    shadow: list = []
    for band in reversed(result.part_band):
        shadow = L.union(shadow, band) if band else shadow
        result.shadow.append(shadow)
    result.shadow.reverse()
    return result


# -- the reach of one cutter -----------------------------------------------------------------------

def _signature(paths) -> int:
    return hash(tuple(tuple(map(tuple, path)) for path in paths))


def _offset(paths, distance_mm: float):
    import pyclipper
    if not paths:
        return []
    job = pyclipper.PyclipperOffset(2.0, OBSTACLE_ARC_MM * L.SCALE)
    job.AddPaths(paths, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
    return job.Execute(distance_mm * L.SCALE)


def _fit(radius: float) -> float:
    return max(radius - FIT_MM, 0.0)


class CutterReach:
    """The material left by one cutter in each layer of one setup."""

    def __init__(self, layers: SetupLayers, cutter: Cutter):
        self.layers, self.cutter = layers, cutter
        self._obstacles: dict[float, list] = {}
        self._offsets: dict[tuple, list] = {}
        self._unions: dict[tuple, list] = {}
        self._clips: dict[tuple, list] = {}
        self._flat: dict[int, list] = {}
        self._eroded: dict[tuple, list] = {}

    def _first_layer_from(self, z: float) -> int:
        """The first layer whose bottom is at or above the height (count if none)."""
        import bisect
        return bisect.bisect_left(self.layers.limits, z - L.Z_MERGE_MM, hi=self.layers.count)

    def obstacle(self, tip: float, box=None, flat: bool = False) -> list:
        """The tool-centre positions where the cutter with its tip at the height touches the part.

        ``box``: only the obstacle near this box is needed. The part sections are clipped to the box grown by three
        times the largest cutter radius: a clip edge changes the obstacle only within that radius of the edge."""
        key = (round(tip, 7), box, flat)
        if key in self._obstacles:
            return self._obstacles[key]
        clip = None
        if box is not None:
            grow = int((3 * max(self.cutter.radius, self.cutter.shank_radius) + 0.01) * L.SCALE)
            clip = [[(box[0] - grow, box[1] - grow), (box[2] + grow, box[1] - grow),
                     (box[2] + grow, box[3] + grow), (box[0] - grow, box[3] + grow)]]
        layers, cutter = self.layers, self.cutter
        c, r, rs = cutter.profile_height, cutter.radius, cutter.shank_radius
        if (c <= 0 or flat) and rs <= r:
            # Flat: the offset of the shadow, built from the top down one layer band at a time (the offset of a
            # union is the union of the offsets), then clipped near the region.
            index = layers.layer_at(tip) if tip > layers.z_low else 0
            if c <= 0 and abs(layers.limits[index] - tip) > L.Z_MERGE_MM and tip > layers.z_low:
                index = self._first_layer_from(tip)
            result = self._clip(self._flat_obstacle(index), clip) if clip is not None else self._flat_obstacle(index)
            self._obstacles[key] = result
            return result
        parts = []
        full = self._first_layer_from(tip + c)  # every layer from here sees the full radius
        if flat:
            # A straight stretch (``_straight``): the part above the tip already holds every section of the profile
            # height, so the full radius on the shadow of the tip layer is the whole obstacle.
            full = layers.layer_at(tip)
        start = layers.layer_at(tip) if tip > layers.z_low else 0
        end = min(full, layers.count) if not flat else start
        if end - start <= 3 * PROFILE_SAMPLES:
            groups = []
            index = start
            while index < end:
                # A run of layers with the same part section: one offset with the radius at the run top.
                last = index
                while last + 1 < end and layers.run[last + 1] == layers.run[index]:
                    last += 1
                groups.append((layers.limits[last + 1], [layers.part_band[index]]))
                index = last + 1
        else:
            # Many thin layers in the profile height: 2 * PROFILE_SAMPLES height bins, each the union of its
            # layers with the radius at the bin top (the radius is at most one bin step too large).
            bins = 2 * PROFILE_SAMPLES
            groups = [(tip + c * (step + 1) / bins, []) for step in range(bins)]
            for index in range(start, end):
                low, high = layers.heights(index)
                first = max(int((low - tip) / c * bins), 0)
                last = min(int(math.ceil((high - tip) / c * bins)), bins)
                for step in range(first, max(last, first + 1)):
                    groups[min(step, bins - 1)][1].append(layers.part_band[index])
            groups = [(min(top, layers.limits[end]), bands) for top, bands in groups]
        # Equal regions in several groups (vertical walls): the radius grows with the height, so only the highest
        # group of each region counts (its offset holds the others).
        highest = {}
        for top, bands in groups:
            if top <= tip + L.Z_MERGE_MM or not any(bands):
                continue
            band = bands[0] if len(bands) == 1 else self._union(bands)
            key = _signature(band)
            if key not in highest or top > highest[key][0]:
                highest[key] = (top, band)
        for top, band in highest.values():
            band = self._clip(band, clip)
            if not band:
                continue
            radius = _fit(cutter.radius_at(top - tip))
            parts.append(self._offset(band, radius) if radius > 0 else band)
        if full < layers.count and layers.shadow[full]:
            parts.append(self._offset(self._clip(layers.shadow[full], clip), _fit(r)))
        if rs > r:
            shank = self._first_layer_from(tip + cutter.flute_length)
            if shank < layers.count and layers.shadow[shank]:
                parts.append(self._offset(self._clip(layers.shadow[shank], clip), _fit(rs)))
        result = L.union([path for item in parts for path in item]) if parts else []
        self._obstacles[key] = result
        return result

    def _offset(self, paths, radius: float) -> list:
        """An offset, kept for equal regions (the equal sections of the layers along vertical walls)."""
        key = (_signature(paths), round(radius, 7))
        if key not in self._offsets:
            self._offsets[key] = _offset(paths, radius)
        return self._offsets[key]

    def _flat_obstacle(self, index: int) -> list:
        layers = self.layers
        if index >= layers.count:
            return []
        if index not in self._flat:
            radius = _fit(self.cutter.radius)
            top = max((k for k in self._flat if k > index), default=layers.count)
            current = self._flat.get(top, [])
            for k in range(top - 1, index - 1, -1):
                band = layers.part_band[k]
                if band and not (k + 1 < layers.count and layers.run[k + 1] == layers.run[k]):
                    current = L.union(current, self._offset(band, radius))
                self._flat[k] = current
        return self._flat[index]

    def _clip(self, paths, clip):
        if clip is None or not paths:
            return paths
        key = (_signature(paths), _signature(clip))
        if key not in self._clips:
            self._clips[key] = L.intersection(paths, clip)
        return self._clips[key]

    def _union(self, regions) -> list:
        key = tuple(_signature(paths) for paths in regions)
        if key not in self._unions:
            self._unions[key] = L.union([path for item in regions for path in item])
        return self._unions[key]

    def _erode(self, tip: float, radius: float, near, box, flat: bool = False) -> list:
        """The obstacle shrunk by the radius, near the region only: the erosion at a point depends only on the
        obstacle within the radius, so the obstacle is clipped to the region box grown by twice the radius."""
        obstacle = self.obstacle(tip, box, flat)
        # The cutter disk reaches up to the mesh chord (1 µm) into the part polygon: the polygon lies that much
        # inside the true part surface, so the disk does not pass the true surface.
        radius = max(radius - L.MESH_DEFLECTION_MM, 0.0)
        if radius <= 0 or not obstacle:
            return obstacle
        box = L.bounds(near)
        grow = int((2 * radius + 0.01) * L.SCALE)
        clip = [[(box[0] - grow, box[1] - grow), (box[2] + grow, box[1] - grow),
                 (box[2] + grow, box[3] + grow), (box[0] - grow, box[3] + grow)]]
        key = (_signature(obstacle), round(radius, 7), _signature(clip))
        if key not in self._eroded:
            self._eroded[key] = _offset(L.intersection(obstacle, clip), -radius)
        return self._eroded[key]

    def left_at(self, index: int, z: float, material) -> list:
        """The material of the layer that the cutter does not reach at the height z."""
        layers, cutter = self.layers, self.cutter
        if not material:
            return []
        if layers.face - z > cutter.reach_depth + 1e-9:
            return material
        if cutter.kind == "drill":
            # A drill works only on the pieces that fit in its largest hole; each piece gets its own small box.
            size = 2 * ((1 + DRILL_HOLE_RATIO) * cutter.radius + 0.01) * L.SCALE
            kept, small = [], []
            for piece in _pieces(material):
                x0, y0, x1, y1 = L.bounds(piece)
                (small if x1 - x0 <= size and y1 - y0 <= size else kept).append(piece)
            for piece in small:
                kept.append(self._left_of(z, piece, L.bounds(piece)))
            return L.union([path for piece in kept for path in piece])
        return self._left_of(z, material, L.bounds(material))

    def _clip_box(self, box):
        grow = int((3 * max(self.cutter.radius, self.cutter.shank_radius) + 0.01) * L.SCALE)
        return [[(box[0] - grow, box[1] - grow), (box[2] + grow, box[1] - grow),
                 (box[2] + grow, box[3] + grow), (box[0] - grow, box[3] + grow)]]

    def _straight(self, z: float, box) -> bool:
        """The part in the profile height below z adds nothing to the shadow of z near the region (and the stock
        bottom is not there): the profile of the cutter meets only what the part above z holds, and the cutter
        acts as a flat cutter with its tip at z."""
        layers, c = self.layers, self.cutter.profile_height
        if z - c < layers.z_low - L.Z_MERGE_MM:
            return False
        index = layers.layer_at(z)
        first = layers.layer_at(z - c)
        if all(layers.run[k] == layers.run[index] for k in range(first, index + 1)):
            return True
        # The part below z (near the region) lies inside the shadow of z: no floor, and no section that reaches
        # further out than the part above (a modelled thread: one pitch above holds every thread section).
        clip = self._clip_box(box)
        shadow = self._clip(layers.shadow[index], clip)
        for k in range(first, index):
            below = self._clip(layers.part_band[k], clip)
            if below and L.opened(L.difference(below, shadow)):
                return False
        return True

    def _left_of(self, z: float, material, box) -> list:
        layers, cutter = self.layers, self.cutter
        c = cutter.profile_height
        straight = c > 0 and self._straight(z, box)
        if c <= 0 or straight:
            tips = [(z, cutter.radius)]
        else:
            # The tip heights: even steps in the profile height, and every horizontal face (a floor) in it.
            import bisect
            heights = {z - c * step / PROFILE_SAMPLES for step in range(PROFILE_SAMPLES + 1)}
            first = bisect.bisect_left(layers.planar, z - c - L.Z_MERGE_MM)
            last = bisect.bisect_right(layers.planar, z + L.Z_MERGE_MM)
            heights.update(layers.planar[first:last])
            # A tip moves to the nearest floor within half a step, else down to a grid of step c / (2 samples) from
            # the stock bottom: the layers share their tip heights and obstacles (a lower tip reaches less, so the
            # grid never claims more reach).
            snap = c / (2 * PROFILE_SAMPLES)
            snapped = set()
            for tip in heights:
                near = bisect.bisect_left(layers.planar, tip)
                options = [layers.planar[k] for k in (near - 1, near) if 0 <= k < len(layers.planar)]
                best = min(options, key=lambda value: abs(value - tip)) if options else None
                if best is not None and abs(best - tip) <= snap / 2 and best <= z + L.Z_MERGE_MM:
                    snapped.add(best)
                else:
                    snapped.add(layers.z_low + math.floor((tip - layers.z_low) / snap + 1e-9) * snap)
            # The lowest tip first: it has the full radius at z and removes most of the region.
            tips = [(tip, cutter.radius_at(z - tip)) for tip in sorted(snapped)
                    if tip >= layers.z_low - c - L.Z_MERGE_MM]
        left = material
        for tip, radius in tips:
            if layers.face - tip > cutter.reach_depth + 1e-9:
                continue
            if cutter.kind == "drill":
                left = L.difference(left, self._drill_reach(tip, radius, left, box, straight))
            else:
                left = L.intersection(left, self._erode(tip, radius, left, box, straight))
            if not left:
                break
        return left

    def _drill_reach(self, tip: float, radius: float, near, box, flat: bool = False) -> list:
        """A drill makes holes: it plunges only where its free centres form a small spot (a hole at most
        ``DRILL_HOLE_RATIO`` larger than the drill), never a pocket or the open stock by plunge milling."""
        obstacle = self.obstacle(tip, box, flat)
        region = L.bounds(near)
        grow = int((2 * self.cutter.radius + 0.01) * L.SCALE)
        clip = [[(region[0] - grow, region[1] - grow), (region[2] + grow, region[1] - grow),
                 (region[2] + grow, region[3] + grow), (region[0] - grow, region[3] + grow)]]
        free = L.difference(clip, obstacle)
        spot = 2 * (DRILL_HOLE_RATIO * self.cutter.radius + FIT_MM) * L.SCALE
        kept = []
        for piece in _pieces(free):
            x0, y0, x1, y1 = L.bounds(piece)
            if x1 - x0 <= spot and y1 - y0 <= spot and x0 > clip[0][0][0] and y0 > clip[0][0][1]                     and x1 < clip[0][2][0] and y1 < clip[0][2][1]:
                kept.extend(piece)
        radius = max(radius - L.MESH_DEFLECTION_MM, 0.0)
        return _offset(kept, radius) if kept and radius > 0 else []

    def left(self, index: int, material) -> list[tuple[float, float, list]]:
        """(z low, z high, left region) bands of the layer: one band for a flat cutter, the profile height
        divided into ``PROFILE_SAMPLES`` bands for a cutter with a profile (each evaluated at its middle)."""
        a, b = self.layers.heights(index)
        c = self.cutter.profile_height
        if c <= 0:
            return [(a, b, self.left_at(index, a, material))]
        cuts = [a + c * step / PROFILE_SAMPLES for step in range(PROFILE_SAMPLES + 1) if a + c * step / PROFILE_SAMPLES < b]
        cuts.append(b)
        return [(low, high, self.left_at(index, (low + high) / 2, material)) for low, high in zip(cuts, cuts[1:])]

    def apply(self, index: int, bands) -> list[tuple[float, float, list]]:
        """The left bands after this cutter, from the bands that the earlier cutters left in the layer."""
        a, b = self.layers.heights(index)
        c = self.cutter.profile_height
        if c <= 0:
            return [(low, high, self.left_at(index, a, region)) for low, high, region in bands]
        cuts = [a + c * step / PROFILE_SAMPLES for step in range(1, PROFILE_SAMPLES + 1)]
        result = []
        for low, high, region in bands:
            inner = [low, *[cut for cut in cuts if low + L.Z_MERGE_MM < cut < high - L.Z_MERGE_MM], high]
            for first, second in zip(inner, inner[1:]):
                result.append((first, second, self.left_at(index, (first + second) / 2, region) if region else []))
        return result


class DrillReach:
    """A drill makes holes (axial plunges). Each hole is a spot of free tool centres, at most
    ``DRILL_HOLE_RATIO`` larger than the drill, under the shadow of a layer: the drill fits down to there.
    For each hole the deepest tip ``t*`` follows from the distance d(z) of the hole axis to the part section of
    each layer below: the point reaches down while ρ(z - t) ≤ d(z) for every layer. The reach of the hole at the
    height z is the disk of radius ρ(z - t*) (the full radius above the point). This is the opening for the
    drill's own profile along one axis: 1D per hole, with one clipped offset per straight hole stretch."""

    def __init__(self, layers: SetupLayers, cutter: Cutter):
        self.layers, self.cutter = layers, cutter
        self.holes: dict[tuple, float] = {}   # (x, y) in nm -> deepest tip height
        self._spots: dict[tuple, list] = {}
        self._clips: dict[tuple, list] = {}
        self._flat: dict[int, list] = {}

    def _flat_obstacle(self, index: int) -> list:
        layers = self.layers
        if index >= layers.count:
            return []
        if index not in self._flat:
            radius = _fit(self.cutter.radius)
            top = max((k for k in self._flat if k > index), default=layers.count)
            current = self._flat.get(top, [])
            for k in range(top - 1, index - 1, -1):
                band = layers.part_band[k]
                if band and not (k + 1 < layers.count and layers.run[k + 1] == layers.run[k]):
                    current = L.union(current, self._offset(band, radius))
                self._flat[k] = current
        return self._flat[index]

    def _clip(self, paths, clip):
        if not paths:
            return paths
        key = (_signature(paths), _signature(clip))
        if key not in self._clips:
            self._clips[key] = L.intersection(paths, clip)
        return self._clips[key]

    def _find(self, index: int, piece) -> None:
        """Holes of the drill in one small piece of a layer: the small spots of free centres under its shadow."""
        layers, r = self.layers, self.cutter.radius
        x0, y0, x1, y1 = L.bounds(piece)
        grow = int((2 * r + 0.01) * L.SCALE)
        clip = [[(x0 - grow, y0 - grow), (x1 + grow, y0 - grow), (x1 + grow, y1 + grow), (x0 - grow, y1 + grow)]]
        shadow = self._clip(layers.shadow[index], clip)
        key = (_signature(shadow), _signature(clip))
        if key not in self._spots:
            free = L.difference(clip, shadow)
            centres = _offset(free, -_fit(r)) if free else []
            spot = 2 * (DRILL_HOLE_RATIO * r + FIT_MM) * L.SCALE
            found = []
            for part in _pieces(centres):
                a, b, c, d = L.bounds(part)
                if c - a <= spot and d - b <= spot and a > clip[0][0][0] + grow / 2 and b > clip[0][0][1] + grow / 2:
                    found.append(((a + c) // 2, (b + d) // 2))
            self._spots[key] = found
        for centre in self._spots[key]:
            rounded = (int(round(centre[0], -4)), int(round(centre[1], -4)))  # 10 µm
            if rounded not in self.holes:
                self.holes[rounded] = self._deepest_tip(centre, index)

    def _distance(self, centre, paths) -> float:
        """The distance (mm) from the point to the boundary of the region (0 inside it)."""
        if not paths:
            return math.inf
        import pyclipper
        point = (int(centre[0]), int(centre[1]))
        best = math.inf
        inside = 0
        for path in paths:
            points = np.asarray(path, float)
            a, b = points, np.roll(points, -1, axis=0)
            ab = b - a
            t = np.clip(((point[0] - a[:, 0]) * ab[:, 0] + (point[1] - a[:, 1]) * ab[:, 1]) /
                        np.maximum((ab ** 2).sum(1), 1e-9), 0, 1)
            nearest = a + ab * t[:, None]
            best = min(best, float(np.sqrt(((nearest - point) ** 2).sum(1)).min()))
            inside += pyclipper.PointInPolygon(point, path) != 0 and (1 if pyclipper.Orientation(path) else -1)
        return 0.0 if inside > 0 else best / L.SCALE

    def _deepest_tip(self, centre, index: int) -> float:
        """The lowest tip height of a plunge on the axis: every layer from the tip up keeps ρ(top - t) within the
        distance of the axis to its part section (less the fit), and the reach depth holds."""
        layers, cutter = self.layers, self.cutter
        c = cutter.profile_height
        lowest = max(layers.z_low - c, layers.face - cutter.reach_depth)
        distances = {}

        def distance(k):
            if k not in distances:
                distances[k] = self._distance(centre, layers.part_band[k])
            return distances[k]

        def fits(tip):
            for k in range(layers.layer_at(tip) if tip > layers.z_low else 0, layers.count):
                top = layers.limits[k + 1]
                if top <= tip:
                    continue
                if top - tip > c + 1e-9 and layers.limits[k] >= tip + c:
                    break  # above the point: the straight part of the drill passed the shadow test of the spot
                if cutter.radius_at(top - tip) - FIT_MM > distance(k):
                    return False
            return True
        high = layers.limits[index]
        if not fits(high):
            return high
        low = lowest
        if fits(low):
            return low
        for _ in range(40):
            middle = (low + high) / 2
            if fits(middle):
                high = middle
            else:
                low = middle
        return high

    def apply(self, index: int, bands):
        a, b = self.layers.heights(index)
        r, c = self.cutter.radius, self.cutter.profile_height
        size = 2 * ((1 + DRILL_HOLE_RATIO) * r + 0.01) * L.SCALE
        # A hole shows as a piece that holds the drill disk (area at least π r², box at most the hole size).
        # Each hole needs one such layer; a piece around a known hole axis is not searched again.
        least = 0.98 * math.pi * r * r
        seen = set()
        for _, _, region in bands:
            for piece in _pieces(region):
                x0, y0, x1, y1 = L.bounds(piece)
                if x1 - x0 > size or y1 - y0 > size or L.area(piece) < least:
                    continue
                key = _signature(piece)
                if key in seen or any(x0 <= x <= x1 and y0 <= y <= y1 for x, y in self.holes):
                    continue
                seen.add(key)
                self._find(index, piece)
        if not self.holes:
            return bands
        cuts = [a + c * step / PROFILE_SAMPLES for step in range(1, PROFILE_SAMPLES + 1)] if c > 0 else []
        result = []
        for low, high, region in bands:
            inner = [low, *[cut for cut in cuts if low + L.Z_MERGE_MM < cut < high - L.Z_MERGE_MM], high]
            for first, second in zip(inner, inner[1:]):
                if not region:
                    result.append((first, second, region))
                    continue
                z = (first + second) / 2
                disks = []
                box = L.bounds(region)
                for (x, y), tip in self.holes.items():
                    radius = min(self.cutter.radius_at(z - tip), r) - L.MESH_DEFLECTION_MM if z > tip else 0.0
                    if radius <= 0 or not (box[0] - r * L.SCALE <= x <= box[2] + r * L.SCALE
                                           and box[1] - r * L.SCALE <= y <= box[3] + r * L.SCALE):
                        continue
                    disks.extend(L.disk_sweep([(x / L.SCALE, y / L.SCALE)], radius))
                result.append((first, second, L.difference(region, L.union(disks)) if disks else region))
        return result


# -- the verdict -----------------------------------------------------------------------------------

def _volume(bands) -> float:
    return sum(L.area(paths) * (high - low) for low, high, paths in bands)


def _bands_by_layer(per_cutter: list[list], material):
    """Intersect the left bands of several cutters of one layer (common band limits)."""
    limits = sorted({value for bands in per_cutter for low, high, _ in bands for value in (low, high)})
    result = []
    for low, high in zip(limits, limits[1:]):
        if high - low <= L.Z_MERGE_MM:
            continue
        middle = (low + high) / 2
        left = material
        for bands in per_cutter:
            region = next(paths for a, b, paths in bands if a - L.Z_MERGE_MM <= middle <= b + L.Z_MERGE_MM)
            left = L.intersection(left, region)
            if not left:
                break
        result.append((low, high, left))
    return result


@dataclass
class SetupVerdict:
    layers: SetupLayers
    cutters: list
    material_mm3: float
    left_mm3: float
    by_cutter: dict
    left: list            # one list of (z low, z high, paths) bands for each layer, all cutters together
    seconds: float
    reach_low: float = -math.inf  # below this height no cutter of the setup cuts (its left is all the material)

    def to_dict(self, *, regions: int = 10):
        return {"setup": self.layers.setup.to_dict(), "layer_count": self.layers.count,
                "z_range_mm": [self.layers.z_low, self.layers.z_high], "setup_face_mm": self.layers.face, "chord_mm": self.layers.chord_mm,
                "material_to_remove_mm3": self.material_mm3, "left_mm3": self.left_mm3,
                "cutters": [cutter.to_dict() for cutter in self.cutters], "left_by_cutter_mm3": self.by_cutter,
                "left_regions": left_regions(self, limit=regions), "seconds": round(self.seconds, 3)}


def setup_verdict(stock_mesh, part_mesh, setup, cutters: Sequence, *, layers: SetupLayers | None = None,
                  full: bool = True) -> SetupVerdict:
    """The material that the cutters do not reach from one setup (each layer, all cutters together).

    ``full``: divide the sloped layers over the whole height (the reference setup of a combination: its left
    volume is the result). Else only down to the deepest cutter reach."""
    started = time.perf_counter()
    cutters = [Cutter.of(cutter) for cutter in cutters]
    deepest = max((cutter.reach_depth for cutter in cutters), default=0.0)
    layers = layers or setup_layers(stock_mesh, part_mesh, setup, reach_depths=[cutter.reach_depth for cutter in cutters],
                                    divide_depth=None if full else deepest)
    # Flat cutters first, the largest first: each next cutter works only on what the earlier ones leave, so the
    # profile cutters (slow) see small regions. ``by_cutter``: the left volume after each cutter, in this order.
    cutters.sort(key=lambda cutter: (cutter.profile_height > 0, -cutter.radius))
    reaches = [DrillReach(layers, cutter) if cutter.kind == "drill" else CutterReach(layers, cutter)
               for cutter in cutters]
    by_cutter = {f"{cutter.kind} D{cutter.diameter:g}" + (f" r{cutter.corner_radius:g}" if cutter.corner_radius else ""): 0.0
                 for cutter in cutters}
    left, material_volume, left_volume = [], 0.0, 0.0
    for index in range(layers.count):
        material = layers.material(index)
        a, b = layers.heights(index)
        material_volume += L.area(material) * (b - a)
        bands = [(a, b, material)] if material else []
        for name, reach in zip(by_cutter, reaches):
            if not any(region for _, _, region in bands):
                break
            bands = [(low, high, L.opened(paths)) for low, high, paths in reach.apply(index, bands)]
            by_cutter[name] += _volume(bands)
        bands = [(low, high, paths) for low, high, paths in bands if paths]
        left.append(bands)
        left_volume += _volume(bands)
    return SetupVerdict(layers, cutters, material_volume, left_volume, by_cutter, left, time.perf_counter() - started,
                        layers.face - deepest if cutters else math.inf)


def left_regions(verdict: SetupVerdict, limit: int = 10):
    """The left material as regions (connected in 2D and in adjacent layers), largest first."""
    rows = []
    for index, bands in enumerate(verdict.left):
        for low, high, paths in bands:
            for piece in _pieces(paths):
                box = L.bounds(piece)
                rows.append([low, high, box, L.area(piece) * (high - low)])
    rows.sort(key=lambda row: row[0])
    regions = []
    for low, high, box, volume in rows:
        for region in regions:
            if abs(region["z"][1] - low) <= 1e-6 and L._overlap(region["box"], box):
                region["z"][1] = high
                region["box"] = (min(region["box"][0], box[0]), min(region["box"][1], box[1]),
                                 max(region["box"][2], box[2]), max(region["box"][3], box[3]))
                region["volume"] += volume
                break
        else:
            regions.append({"z": [low, high], "box": box, "volume": volume})
    regions.sort(key=lambda region: -region["volume"])
    return [{"z_mm": [round(region["z"][0], 4), round(region["z"][1], 4)],
             "xy_box_mm": [round(value / L.SCALE, 4) for value in region["box"]],
             "volume_mm3": region["volume"]} for region in regions[:limit]]


def _pieces(paths):
    """The outer loops of a region, each with its holes."""
    import pyclipper
    if not paths:
        return []
    clipper = pyclipper.Pyclipper()
    clipper.AddPaths(paths, pyclipper.PT_SUBJECT, True)
    tree = clipper.Execute2(pyclipper.CT_UNION, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO)
    pieces = []

    def visit(node):
        for child in node.Childs:
            if child.IsHole:
                visit(child)
                continue
            pieces.append([child.Contour, *[hole.Contour for hole in child.Childs]])
            for hole in child.Childs:
                visit(hole)
    visit(tree)
    return pieces


# -- several setups --------------------------------------------------------------------------------

def _axis_of(vector) -> tuple[int, float]:
    vector = np.asarray(vector, float)
    axis = int(np.argmax(np.abs(vector)))
    if abs(abs(vector[axis]) - 1.0) > 1e-9:
        raise ValueError("The setup combination requires setups along the part axes.")
    return axis, float(np.sign(vector[axis]))


def _line_intervals(paths, coordinate: int, value: float):
    """The intervals of the region on the line (x or y of the paths) = value (even-odd crossings)."""
    other = 1 - coordinate
    crossings = []
    target = value * L.SCALE
    for path in paths:
        points = np.asarray(path, float)
        a, b = points, np.roll(points, -1, axis=0)
        ca, cb = a[:, coordinate], b[:, coordinate]
        cross = ((ca <= target) & (cb > target)) | ((cb <= target) & (ca > target))
        if not np.any(cross):
            continue
        t = (target - ca[cross]) / (cb[cross] - ca[cross])
        crossings.extend((a[cross, other] + t * (b[cross, other] - a[cross, other])).tolist())
    crossings.sort()
    return list(zip(crossings[0::2], crossings[1::2]))


def section_in(verdict: SetupVerdict, reference: Setup, z: float):
    """The left material of another setup on the plane z (reference setup frame), in reference xy paths."""
    other = verdict.layers.setup
    # Reference axes in the other frame: column k of (other^T reference).
    relation = other.rotation.T @ reference.rotation
    normal_axis, normal_sign = _axis_of(relation[:, 2])
    x_axis, x_sign = _axis_of(relation[:, 0])
    y_axis, y_sign = _axis_of(relation[:, 1])
    # A point of the other frame (p0, p1, p2) has reference coordinates x = x_sign*p[x_axis], y = y_sign*p[y_axis].
    level = normal_sign * z  # the plane: p[normal_axis] = level
    pieces = []
    reach_low = verdict.reach_low
    if normal_axis == 2:
        if level < reach_low - L.Z_MERGE_MM:
            return None  # no cutter of this setup reaches the plane: no constraint
        index = verdict.layers.layer_at(level)
        for low, high, paths in verdict.left[index]:
            if low - 1e-9 <= level <= high + 1e-9:
                for path in paths:
                    mapped = []
                    for point in path:
                        p = (point[0], point[1])
                        mapped.append((int(x_sign * p[x_axis]), int(y_sign * p[y_axis])))
                    pieces.append(mapped)
        return L.union(pieces)
    rects = []
    in_plane = 1 - normal_axis

    def rectangle(start, end, low, high):
        corners = []
        for along, height in ((start, low), (end, low), (end, high), (start, high)):
            p = [0.0, 0.0, 0.0]
            p[in_plane] = along / L.SCALE
            p[2] = height
            corners.append((int(round(x_sign * p[x_axis] * L.SCALE)), int(round(y_sign * p[y_axis] * L.SCALE))))
        return corners
    if reach_low > verdict.layers.z_low:
        # Below the reach of this setup: all the material stays (one rectangle over the whole plane width).
        wide = 1e5 * L.SCALE
        rects.append(rectangle(-wide, wide, verdict.layers.z_low - 1.0, min(reach_low, verdict.layers.z_high + 1.0)))
    for bands in verdict.left:
        for low, high, paths in bands:
            if high <= reach_low + L.Z_MERGE_MM:
                continue
            for start, end in _line_intervals(paths, normal_axis, level):
                # The rectangle: the in-plane coordinate of the other frame from start to end, z from low to high.
                rects.append(rectangle(start, end, low, high))
    return L.union(rects)


def combined_left(verdicts: list[SetupVerdict], reference: int = 0):
    """The material that no setup reaches: the left material of the reference setup that every other setup
    also leaves (2D, in the reference layers). Returns (volume, bands for each reference layer)."""
    base = verdicts[reference]
    others = [verdict for position, verdict in enumerate(verdicts) if position != reference]
    total, result = 0.0, []
    for bands in base.left:
        combined = []
        for low, high, paths in bands:
            left = paths
            for verdict in others:
                if not left:
                    break
                section = section_in(verdict, base.layers.setup, (low + high) / 2)
                if section is not None:
                    left = L.intersection(left, section)
            left = L.opened(left)
            combined.append((low, high, left))
            total += L.area(left) * (high - low)
        result.append(combined)
    return total, result


def part_verdict(stock_shape, part_shape, setups: Iterable, cutters, *, reference: int = 0, extra_shapes=()) -> dict:
    """The milling verdict of a part: for each setup the material its cutters leave, and the material that no
    setup reaches. ``cutters``: one list for all setups, or a dict {setup name: list}."""
    started = time.perf_counter()
    stock_mesh = L.SolidMesh.of(stock_shape)
    part_mesh = L.SolidMesh.of(part_shape)
    extra = [L.SolidMesh.of(shape) for shape in extra_shapes]
    mesh_seconds = time.perf_counter() - started
    verdicts = []
    for position, setup in enumerate(setups):
        setup = Setup.of(setup)
        tools = cutters.get(setup.name, []) if isinstance(cutters, dict) else cutters
        layers = setup_layers(stock_mesh, part_mesh, setup, extra_meshes=extra,
                              reach_depths=[Cutter.of(tool).reach_depth for tool in tools]) if extra else None
        verdicts.append(setup_verdict(stock_mesh, part_mesh, setup, tools, layers=layers, full=position == reference))
    total, bands = combined_left(verdicts, reference) if verdicts else (None, [])
    return {"schema": "subcad.milling_opening.v1", "method": "morphological_opening_by_layer",
            "contact_tolerance_mm": L.CONTACT_MM, "arc_tolerance_mm": L.ARC_TOLERANCE_MM,
            "setups": [verdict.to_dict() for verdict in verdicts], "left_after_all_setups_mm3": total,
            "can_make": total is not None and total <= 1e-6, "mesh_seconds": round(mesh_seconds, 3),
            "seconds": round(time.perf_counter() - started, 3), "_verdicts": verdicts, "_combined": bands}
