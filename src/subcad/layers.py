"""2.5D material layers for fixed-axis milling checks.

A milling setup has one fixed tool axis. In the setup frame, a flat end mill at tip height ``z``
removes, from each material layer between ``z`` and the top of its flutes, the 2D region that its
disk sweeps along the move. This module keeps the material of one setup as horizontal layers.
Each layer is one 2D polygon set (pyclipper, integer scale 1e6: 1 unit = 1 nm) over a height
interval. The checks use only 2D polygon operations, areas and layer heights:

* removed volume of a pass: the area of (layer material ∩ sweep) × layer height, summed;
* material left in the operation region and material removed outside it: 2D differences with a
  contact tolerance of 1 micrometre, times the layer height.

The incoming material comes from one triangle mesh of the solid. A plane section of the mesh is
plain arithmetic: each crossed triangle gives one segment, and the segments join at the mesh
edges that they cross. Layer limits are the cutting levels and the mesh vertex heights. A layer
that a sloped or curved face crosses is divided into thinner layers (``SLOPED_LAYER_MM``); each
layer uses its mid-height section. No solid Boolean, no OCC section and no point-in-solid test
runs here.
"""
from __future__ import annotations

import math
from collections import defaultdict
from typing import Iterable, Sequence

import numpy as np
import pyclipper

# Integer scale: 1 unit = 1 nanometre.
SCALE = 1_000_000
# Largest distance between a true arc and its chord in the cutter sweep polygons (mm).
ARC_TOLERANCE_MM = 0.00025
# Largest distance between a true arc and its chord in the operation region polygon (mm). Material left
# along a curved wall is often a thin crescent; a coarser chord would cut a visible part of it off.
TARGET_TOLERANCE_MM = 0.00001
# Material thinner than this, along a cut boundary, counts as removed (contact tolerance, mm).
CONTACT_MM = 0.001
# Mesh chord limit of the incoming solid (mm) and the angle limit between two chords of one curve (rad).
MESH_DEFLECTION_MM = 0.001
MESH_ANGLE_RAD = 0.1
# Height of the thinner layers where a sloped or curved face crosses a layer (mm).
SLOPED_LAYER_MM = 0.1
# Largest number of layers of one operation; above it the sloped layers are made thicker.
MAX_LAYERS = 400
# Outline tolerance of the material shown after each pass (operation player only, not the checks) (mm).
DISPLAY_MM = 0.0005
# Two heights closer than this are one layer limit (mm).
Z_MERGE_MM = 1e-7


Paths = list  # list of closed integer paths [(x, y), ...]


# -- 2D polygon operations -------------------------------------------------------------------------

def to_paths(loops_mm: Iterable[Sequence[Sequence[float]]]) -> Paths:
    # np.rint rounds half to even, as round() does.
    return [[tuple(point) for point in np.rint(np.asarray(loop, float)[:, :2] * SCALE).astype(np.int64).tolist()]
            for loop in loops_mm if len(loop) >= 3]


def _clip(operation, subject: Paths, clip: Paths | None = None, *, fill=pyclipper.PFT_NONZERO) -> Paths:
    if not subject:
        return []
    clipper = pyclipper.Pyclipper()
    try:
        clipper.AddPaths(subject, pyclipper.PT_SUBJECT, True)
    except pyclipper.ClipperException:
        return []
    if clip:
        try:
            clipper.AddPaths(clip, pyclipper.PT_CLIP, True)
        except pyclipper.ClipperException:
            if operation == pyclipper.CT_INTERSECTION:
                return []
            if operation == pyclipper.CT_DIFFERENCE:
                return union(subject)
    return clipper.Execute(operation, fill, fill)


def union(subject: Paths, other: Paths | None = None) -> Paths:
    return _clip(pyclipper.CT_UNION, subject + (other or []))


def even_odd(subject: Paths) -> Paths:
    """The region of loops by the even-odd rule (a closed solid's section loops)."""
    return _clip(pyclipper.CT_UNION, subject, fill=pyclipper.PFT_EVENODD)


def intersection(subject: Paths, clip: Paths) -> Paths:
    if not subject or not clip:
        return []
    return _clip(pyclipper.CT_INTERSECTION, subject, clip)


def difference(subject: Paths, clip: Paths) -> Paths:
    if not subject:
        return []
    if not clip:
        return subject
    return _clip(pyclipper.CT_DIFFERENCE, subject, clip)


def display_paths(paths: Paths) -> Paths:
    """The region with the vertices that change its outline by less than ``DISPLAY_MM`` removed (player states)."""
    if not paths:
        return []
    return [path for path in pyclipper.CleanPolygons(paths, DISPLAY_MM * SCALE) if len(path) >= 3]


def offset(paths: Paths, distance_mm: float) -> Paths:
    if not paths:
        return []
    job = pyclipper.PyclipperOffset(2.0, ARC_TOLERANCE_MM * SCALE)
    job.AddPaths(paths, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
    return job.Execute(distance_mm * SCALE)


def area(paths: Paths) -> float:
    """Net area in mm² (outer loops positive, holes negative)."""
    return sum(pyclipper.Area(path) for path in paths) / (SCALE * SCALE)


def bounds(paths: Paths):
    if not paths:
        return None
    xs = [x for path in paths for x, _ in path]
    ys = [y for path in paths for _, y in path]
    return min(xs), min(ys), max(xs), max(ys)


def _overlap(first, second) -> bool:
    return first is not None and second is not None and not (
        first[2] < second[0] or second[2] < first[0] or first[3] < second[1] or second[3] < first[1])


def disk_sweep(points_mm: Sequence[Sequence[float]], radius_mm: float, *, closed: bool = False) -> Paths:
    """The region that a disk of the radius covers along the polyline (2D Minkowski sum).

    The arc vertices lie on the cutter circle; each chord is at most the arc tolerance inside it.
    ``paths_to_faces`` rebuilds these chords as exact arcs for the B-Rep result."""
    points = [(int(round(x * SCALE)), int(round(y * SCALE))) for x, y, *_ in points_mm]
    clean = [points[0]]
    for point in points[1:]:
        if point != clean[-1]:
            clean.append(point)
    if closed and len(clean) > 1 and clean[0] == clean[-1]:
        clean.pop()
    job = pyclipper.PyclipperOffset(2.0, ARC_TOLERANCE_MM * SCALE)
    kind = pyclipper.ET_CLOSEDLINE if closed and len(clean) >= 3 else pyclipper.ET_OPENROUND
    job.AddPath(clean, pyclipper.JT_ROUND, kind)
    return job.Execute(radius_mm * SCALE)


def face_paths(face, tolerance_mm: float = TARGET_TOLERANCE_MM) -> Paths:
    """The region of a planar CadQuery face (or the faces of a shape) at z = 0 as integer paths."""
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.BRepTools import BRepTools_WireExplorer
    from OCP.GCPnts import GCPnts_QuasiUniformDeflection
    from OCP.GeomAbs import GeomAbs_Line
    from OCP.TopAbs import TopAbs_REVERSED

    loops = []
    for single in face.Faces():
        for wire in single.Wires():
            explorer, points = BRepTools_WireExplorer(wire.wrapped), []
            while explorer.More():
                edge = explorer.Current()
                curve = BRepAdaptor_Curve(edge)
                if curve.GetType() == GeomAbs_Line:
                    values = [curve.Value(curve.FirstParameter()), curve.Value(curve.LastParameter())]
                else:
                    sampler = GCPnts_QuasiUniformDeflection(curve, tolerance_mm)
                    if not sampler.IsDone():
                        raise ValueError("A profile edge cannot be divided into chords.")
                    values = [sampler.Value(index) for index in range(1, sampler.NbPoints() + 1)]
                xy = [(point.X(), point.Y()) for point in values]
                if edge.Orientation() == TopAbs_REVERSED:
                    xy.reverse()
                points.extend(xy if not points else xy[1:])
                explorer.Next()
            loops.append(points)
    return even_odd(to_paths(loops))


def _segment_arcs(points: np.ndarray, centers: np.ndarray, tree, radius: float) -> np.ndarray:
    """For each segment i -> i+1 of a closed loop: the index of the cutter circle that it is a chord of, or -1.

    A chord has both ends and its middle in the band from five times the arc tolerance inside the circle to the circle
    (integer rounding: a few nm). Two overlapping polygonal arcs of one circle meet inside this band."""
    following = np.roll(points, -1, axis=0)
    middle = (points + following) / 2
    # pyclipper can close a circle with a chord deeper than its arc tolerance: allow five times the tolerance.
    near, band = 4.0, 5 * ARC_TOLERANCE_MM * SCALE + 4.0
    result = np.full(len(points), -1)

    def on_band(values, center):
        distance = np.linalg.norm(values - center, axis=1) - radius
        return (distance <= near) & (distance >= -band)
    for candidate in (tree.query(points)[1], tree.query(following)[1]):
        center = centers[candidate]
        chord = on_band(points, center) & on_band(following, center) & on_band(middle, center)
        result = np.where((result < 0) & chord, candidate, result)
    return result


def _on_circles(point, first, second, radius):
    """The point on the circle around ``first`` nearest to ``point``; on both circles where they meet."""
    if second is not None:
        delta = second - first
        distance = float(np.linalg.norm(delta))
        if 1e-9 < distance < 2 * radius:
            along = delta / distance
            middle = first + along * (distance / 2)
            height = math.sqrt(max(radius * radius - distance * distance / 4, 0.0))
            normal = np.array((-along[1], along[0]))
            options = (middle + normal * height, middle - normal * height)
            best = min(options, key=lambda option: float(np.linalg.norm(option - point)))
            # Two chords meet up to the arc tolerance inside both circles; where the circles cross at a small
            # angle, the true crossing lies further along the boundary (tolerance / sin(half angle)).
            if np.linalg.norm(best - point) <= 0.05 * SCALE:
                return best
    offset = point - first
    return first + offset * (radius / np.linalg.norm(offset))


def _wire(contour, arcs=None):
    """An OCC wire of one closed integer loop. With ``arcs`` = (centers, tree, radius), each run of chords
    of one cutter circle becomes one exact circular edge."""
    from OCP.BRep import BRep_Builder
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeWire
    from OCP.Geom import Geom_Circle
    from OCP.gp import gp_Ax2, gp_Circ, gp_Dir, gp_Pnt
    from OCP.TopoDS import TopoDS_Vertex

    if arcs is None:
        polygon = BRepBuilderAPI_MakePolygon()
        for x, y in contour:
            polygon.Add(gp_Pnt(x / SCALE, y / SCALE, 0.0))
        polygon.Close()
        return polygon.Wire()
    centers, tree, radius = arcs
    # Two vertices a few nanometres apart make an edge shorter than the vertex tolerance: drop one.
    contour = pyclipper.CleanPolygon(contour, 20)
    points = np.asarray(contour, float)
    count = len(points)
    owner = _segment_arcs(points, centers, tree, radius)
    if owner[0] >= 0 and np.all(owner == owner[0]):
        center = centers[owner[0]] / SCALE
        normal = 1.0 if pyclipper.Orientation(contour) else -1.0
        circle = gp_Circ(gp_Ax2(gp_Pnt(center[0], center[1], 0.0), gp_Dir(0, 0, normal)), radius / SCALE)
        return BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(circle).Edge()).Wire()
    builder = BRep_Builder()
    # Start at a change of edge, so that no arc wraps around the start of the loop.
    start = next(index for index in range(count) if owner[index] < 0 or owner[index] != owner[index - 1])
    order = [(start + index) % count for index in range(count)]
    vertices = {}

    def vertex(index):
        if index not in vertices:
            point = points[index]
            before, after = owner[index - 1], owner[index]
            circles = [centers[value] for value in (before, after) if value >= 0]
            if circles:
                point = _on_circles(point, circles[0], circles[1] if len(circles) == 2 and before != after else None, radius)
            made = TopoDS_Vertex()
            builder.MakeVertex(made, gp_Pnt(point[0] / SCALE, point[1] / SCALE, 0.0), 1e-5)
            vertices[index] = made
        return vertices[index]
    wire = BRepBuilderAPI_MakeWire()
    position = 0
    while position < count:
        first = order[position]
        circle = owner[first]
        if circle < 0:
            last = order[(position + 1) % count]
            edge = BRepBuilderAPI_MakeEdge(vertex(first), vertex(last))
            position += 1
        else:
            span = 1
            while position + span < count and owner[order[position + span]] == circle:
                span += 1
            last = order[(position + span) % count]
            center = centers[circle]
            run = [points[order[(position + k) % count]] - center for k in range(span + 1)]
            turn = sum(a[0] * b[1] - a[1] * b[0] for a, b in zip(run, run[1:]))
            axis = gp_Ax2(gp_Pnt(center[0] / SCALE, center[1] / SCALE, 0.0), gp_Dir(0, 0, 1.0 if turn > 0 else -1.0))
            edge = BRepBuilderAPI_MakeEdge(Geom_Circle(axis, radius / SCALE), vertex(first), vertex(last))
            position += span
        if not edge.IsDone():
            raise ValueError("The cutter region boundary cannot form an edge.")
        wire.Add(edge.Edge())
    if not wire.IsDone():
        raise ValueError("The cutter region boundary does not form a closed wire.")
    return wire.Wire()


def cutter_arcs(centers_mm, radius_mm: float):
    """The cutter circles (tool-centre points and radius) that ``paths_to_faces`` rebuilds as exact arcs."""
    from scipy.spatial import cKDTree
    if centers_mm is None or not len(centers_mm) or not radius_mm:
        return None
    centers = np.unique(np.round(np.asarray(centers_mm, float)[:, :2] * SCALE), axis=0)
    return centers, cKDTree(centers), radius_mm * SCALE


def paths_to_faces(paths: Paths, *, arcs=None):
    """Planar OCC faces at z = 0 for the polygon region (outer loops with their holes).

    With ``arcs`` (``cutter_arcs``), the chords of each cutter circle become one exact arc."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace

    if not paths:
        return []
    clipper = pyclipper.Pyclipper()
    clipper.StrictlySimple = True  # no touching vertices: each loop is one valid OCC wire
    clipper.AddPaths(paths, pyclipper.PT_SUBJECT, True)
    tree = clipper.Execute2(pyclipper.CT_UNION, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO)
    faces = []

    def visit(node):
        for child in node.Childs:
            if child.IsHole:
                visit(child)
                continue
            maker = BRepBuilderAPI_MakeFace(_wire(child.Contour, arcs), True)
            for hole in child.Childs:
                maker.Add(_wire(hole.Contour, arcs))
            if not maker.IsDone():
                raise ValueError("The polygon region cannot form a planar face.")
            faces.append(maker.Face())
            for hole in child.Childs:
                visit(hole)
    visit(tree)
    return faces


def prisms(paths: Paths, z_low: float, z_high: float, *, arcs=None, single: bool = False):
    """Solids of the polygon region between two heights (extruded faces, no Boolean).

    ``single``: one compound of all the faces' prisms (one extrusion call), for display states."""
    import cadquery as cq
    from OCP.BRep import BRep_Builder
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
    from OCP.TopoDS import TopoDS_Compound
    from OCP.gp import gp_Trsf, gp_Vec

    move = gp_Trsf()
    move.SetTranslation(gp_Vec(0, 0, z_low))
    faces = paths_to_faces(paths, arcs=arcs)
    if single and faces:
        compound, builder = TopoDS_Compound(), BRep_Builder()
        builder.MakeCompound(compound)
        for face in faces:
            builder.Add(compound, face)
        faces = [compound]
    solids = []
    for face in faces:
        prism = BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, z_high - z_low), False, True).Shape()
        solids.append(cq.Shape.cast(BRepBuilderAPI_Transform(prism, move, False).Shape()))
    return solids


# -- the triangle mesh of a solid ------------------------------------------------------------------

class SolidMesh:
    """The boundary triangles of a solid; faces share their edge nodes."""

    def __init__(self, vertices: np.ndarray, triangles: np.ndarray):
        self.vertices = vertices
        self.triangles = triangles
        corners = vertices[triangles]
        normal = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
        length = np.linalg.norm(normal, axis=1)
        length[length == 0] = 1.0
        self.normal_z = normal[:, 2] / length
        heights = corners[:, :, 2]
        self.low, self.high = heights.min(1), heights.max(1)

    @classmethod
    def of(cls, shape, deflection: float = MESH_DEFLECTION_MM, angle: float = MESH_ANGLE_RAD) -> "SolidMesh":
        from OCP.BRep import BRep_Tool
        from OCP.BRepMesh import BRepMesh_IncrementalMesh
        from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopLoc import TopLoc_Location
        from OCP.TopoDS import TopoDS

        # Mesh a copy: OCCT keeps a triangulation on the faces, and the process material shares its faces.
        # The faces mesh in parallel (each face on its own; the result is the same as one by one).
        native = shape.copy().wrapped
        BRepMesh_IncrementalMesh(native, deflection, False, angle, True)
        points, triangles, offset_index = [], [], 0
        explorer = TopExp_Explorer(native, TopAbs_FACE)
        while explorer.More():
            face = TopoDS.Face(explorer.Current())
            location = TopLoc_Location()
            triangulation = BRep_Tool.Triangulation_s(face, location)
            explorer.Next()
            if triangulation is None:
                continue
            transform = location.Transformation()
            count = triangulation.NbNodes()
            for node in range(1, count + 1):
                point = triangulation.Node(node).Transformed(transform)
                points.append((point.X(), point.Y(), point.Z()))
            reverse = face.Orientation() == TopAbs_REVERSED
            for index in range(1, triangulation.NbTriangles() + 1):
                a, b, c = triangulation.Triangle(index).Get()
                triangles.append((offset_index + a - 1, offset_index + c - 1, offset_index + b - 1) if reverse else
                                 (offset_index + a - 1, offset_index + b - 1, offset_index + c - 1))
            offset_index += count
        if not triangles:
            raise ValueError("The material has no surface mesh for the layer check.")
        vertices = np.array(points)
        # Faces share their edge nodes: equal points become one vertex, so sections join across faces.
        keys = np.round(vertices / 1e-7).astype(np.int64)
        _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
        return cls(vertices[first], inverse.reshape(-1)[np.array(triangles)])

    def section(self, level: float) -> Paths:
        """The material region of the plane z = level (even-odd region of the joined loops)."""
        crossing = np.nonzero((self.low < level) & (self.high > level))[0]
        if len(crossing) == 0:
            return []
        tris = self.triangles[crossing]
        h = self.vertices[:, 2] - level
        th = h[tris]
        # A vertex on the plane counts as below it: every crossed edge then has one end above.
        above = th > 0
        edges = np.stack([tris[:, [0, 1]], tris[:, [1, 2]], tris[:, [2, 0]]], 1)
        flags = np.stack([above[:, [0, 1]], above[:, [1, 2]], above[:, [2, 0]]], 1)
        cut = flags[:, :, 0] != flags[:, :, 1]
        valid = cut.sum(1) == 2
        edges, cut = edges[valid], cut[valid]
        pick = np.argsort(~cut, axis=1, kind="stable")[:, :2]
        rows = np.arange(len(edges))[:, None]
        chosen = np.sort(edges[rows, pick], axis=2)
        p0, p1 = self.vertices[chosen[..., 0]], self.vertices[chosen[..., 1]]
        h0, h1 = h[chosen[..., 0]], h[chosen[..., 1]]
        t = (h0 / (h0 - h1))[..., None]
        points = (p0 + t * (p1 - p0))[..., :2]
        keys = chosen[..., 0].astype(np.int64) * len(self.vertices) + chosen[..., 1]
        loops = _join(keys, points)
        return even_odd(to_paths(loops))


def _join(keys: np.ndarray, points: np.ndarray):
    """Join segments (one for each crossed triangle) into closed loops at their shared mesh edges.

    Each crossed mesh edge is the end of exactly two segments in a closed mesh; ``partner`` links the two ends."""
    count = len(keys)
    if count == 0:
        return []
    _, inverse, occurrences = np.unique(keys.reshape(-1), return_inverse=True, return_counts=True)
    if np.any(occurrences != 2):
        return _join_general(keys, points)  # a mesh edge with more than two crossings (touching solids)
    order = np.argsort(inverse, kind="stable")
    partner = np.empty(2 * count, np.int64)
    partner[order[0::2]] = order[1::2]
    partner[order[1::2]] = order[0::2]
    # A state 2 s + e: segment s entered at its end e. The next state leaves s at its other end and enters the
    # segment that shares that mesh edge. The states form cycles; each loop is two cycles (both directions), and the
    # cycle that holds its first segment entered at end 0 (an even smallest state) is kept. Pointer jumping gives
    # each state its cycle (smallest state) and its step from there, all in numpy.
    states = 2 * count
    following = partner[np.arange(states) ^ 1]
    label = np.arange(states)
    jump = following.copy()
    span = 1
    while span < states:
        label = np.minimum(label, label[jump])
        jump = jump[jump]
        span *= 2
    keep = (label % 2 == 0)
    previous = np.empty(states, np.int64)
    previous[following] = np.arange(states)
    leader = label == np.arange(states)
    rank = np.where(leader, 0, 1)
    jump = np.where(leader, np.arange(states), previous)
    span = 1
    while span < states:
        rank = rank + rank[jump]
        jump = jump[jump]
        span *= 2
    chosen = np.nonzero(keep)[0]
    chosen = chosen[np.lexsort((rank[chosen], label[chosen]))]
    ends = points.reshape(-1, 2)[chosen]
    cuts = np.nonzero(np.diff(label[chosen]))[0] + 1
    return [list(map(tuple, part.tolist())) for part in np.split(ends, cuts)]


def _join_general(keys: np.ndarray, points: np.ndarray):
    """Join segments at their shared mesh edges when an edge has more than two segments."""
    point_of, neighbours = {}, defaultdict(list)
    for index in range(len(keys)):
        k0, k1 = int(keys[index, 0]), int(keys[index, 1])
        point_of[k0] = points[index, 0]
        point_of[k1] = points[index, 1]
        neighbours[k0].append((k1, index))
        neighbours[k1].append((k0, index))
    used = np.zeros(len(keys), bool)
    loops = []
    for start in list(neighbours):
        if all(used[i] for _, i in neighbours[start]):
            continue
        path, current, closed = [start], start, False
        while True:
            step = next(((k, i) for k, i in neighbours[current] if not used[i]), None)
            if step is None:
                break
            used[step[1]] = True
            current = step[0]
            if current == start:
                closed = True
                break
            path.append(current)
        if not closed:
            raise ValueError("The material mesh is not closed. The layer check cannot read its sections.")
        loops.append([tuple(point_of[k]) for k in path])
    return loops


# -- layers ----------------------------------------------------------------------------------------

class Layer:
    __slots__ = ("z_low", "z_high", "material", "box", "group")

    def __init__(self, z_low: float, z_high: float, material: Paths, group: int = 0):
        self.z_low, self.z_high = z_low, z_high
        self.group = group  # the layers of one interval between two layer limits share a group
        self.material = material
        self.box = bounds(material)

    @property
    def height(self) -> float:
        return self.z_high - self.z_low

    def set(self, material: Paths):
        self.material = material
        self.box = bounds(material)


def _merged(values: Iterable[float]) -> list[float]:
    result = []
    for value in sorted(values):
        if not result or value - result[-1] > Z_MERGE_MM:
            result.append(value)
    return result


class LayeredMaterial:
    """The material of one setup between two heights, as 2D layers (setup frame, tool axis +Z)."""

    def __init__(self, layers: list[Layer], z_low: float, z_high: float, sloped_layer_mm: float):
        self.layers = layers
        self.z_low, self.z_high = z_low, z_high
        self.sloped_layer_mm = sloped_layer_mm

    @classmethod
    def of_solid(cls, shape, z_low: float, z_high: float, levels: Iterable[float] = (),
                 mesh: SolidMesh | None = None) -> "LayeredMaterial":
        """Layers of the solid between z_low and z_high, with a layer limit at each cutting level."""
        mesh = mesh or SolidMesh.of(shape)
        if z_high - z_low <= Z_MERGE_MM:
            return cls([], z_low, z_high, SLOPED_LAYER_MM)
        heights = mesh.vertices[:, 2]
        inside = heights[(heights > z_low + Z_MERGE_MM) & (heights < z_high - Z_MERGE_MM)]
        limits = _merged([z_low, z_high, *[level for level in levels if z_low < level < z_high], *inside.tolist()])
        sloped = np.abs(mesh.normal_z) > 1e-9
        sloped &= np.abs(mesh.normal_z) < 1 - 1e-12
        low, high = mesh.low[sloped], mesh.high[sloped]
        intervals = []
        for first, second in zip(limits, limits[1:]):
            crossed = bool(np.any((low < second - Z_MERGE_MM) & (high > first + Z_MERGE_MM)))
            intervals.append((first, second, crossed))
        step = SLOPED_LAYER_MM
        while True:
            count = sum(max(1, math.ceil((b - a) / step - 1e-9)) if crossed else 1 for a, b, crossed in intervals)
            if count <= MAX_LAYERS:
                break
            step *= 1.5
        layers = []
        for group, (first, second, crossed) in enumerate(intervals):
            parts = max(1, math.ceil((second - first) / step - 1e-9)) if crossed else 1
            for index in range(parts):
                a = first + (second - first) * index / parts
                b = first + (second - first) * (index + 1) / parts
                layers.append(Layer(a, b, mesh.section((a + b) / 2), group))
        return cls(layers, z_low, z_high, step)

    def copy_material(self) -> list[Paths]:
        return [layer.material for layer in self.layers]

    def volume(self) -> float:
        return sum(area(layer.material) * layer.height for layer in self.layers)

    def remove(self, sweep: Paths, z_tip: float, z_top: float = math.inf, collect: list | None = None) -> float:
        """Remove the sweep region from the layers between the tool tip and the flute top.

        Returns the removed volume (mm³). ``collect`` gets (z low, z high, removed region) of each layer."""
        box = bounds(sweep)
        removed = 0.0
        for layer in self.layers:
            if layer.z_low < z_tip - Z_MERGE_MM or layer.z_high > z_top + Z_MERGE_MM:
                continue
            if not _overlap(layer.box, box):
                continue
            cut = intersection(layer.material, sweep)
            if not cut:
                continue
            removed += area(cut) * layer.height
            if collect is not None:
                collect.append((layer.z_low, layer.z_high, cut))
            layer.set(difference(layer.material, sweep))
        return removed


def opened(paths: Paths, width_mm: float | None = None) -> Paths:
    """The region without its parts thinner than ``width_mm`` (2D opening: shrink, then grow by half the width)."""
    if not paths:
        return []
    width_mm = CONTACT_MM if width_mm is None else width_mm
    return offset(offset(paths, -width_mm / 2), width_mm / 2)


def region_comparison(before: list[Paths], after: LayeredMaterial, target: Paths, z_low: float, z_high: float):
    """Material left in the target region and material removed outside it (mm³), by layers.

    ``before``: the layer materials before the operation (same layer limits as ``after``).
    A region thinner than the contact tolerance (polygon chords of two arcs) does not count."""
    remaining = overcut = 0.0
    for material, layer in zip(before, after.layers):
        removal = difference(material, layer.material)
        if removal:
            outside = opened(difference(removal, target))
            if outside:
                overcut += area(outside) * layer.height
        if layer.z_low >= z_low - Z_MERGE_MM and layer.z_high <= z_high + Z_MERGE_MM:
            left = opened(intersection(layer.material, target))
            if left:
                remaining += area(left) * layer.height
    return max(0.0, remaining), max(0.0, overcut)
