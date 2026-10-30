"""Level roughing: the waste above each planar level of a setup, before its features (docs/LEVEL_ROUGHING.md).

A stepped part has its features on several planar levels. Before a hole in a lower section can be drilled from its
own level, the waste above that level must go. For one setup (one fixed tool axis) and a flat end mill of radius r:

    silhouette = the part outline along the tool axis (the union of all part sections, holes filled)
    shadow(z)  = the part sections at and above the height z, holes and closed pockets filled
    region(z)  = the incoming material ∩ silhouette − closing(shadow(z), r)

The closing (``milling_opening``) is what a disk of radius r cannot reach without touching the part at or above z,
so the cutter never gouges the part and never goes under an overhang. The silhouette keeps the outside profile for its
own operation; the filled shadow keeps the holes and the closed pockets for theirs. The shadow grows downward, so the
region of a lower level is inside the region of the level above: one region for each band between two planar heights
(the region at the bottom of the band) is safe for the whole band.

The passes are horizontal: a raster of the free cutter centres and their outline, at each stepdown level of each
band, down to the reach of the cutter. ``milling_layers.LayerReplay`` removes the material (2D layers), writes the
material ledger and makes the B-Rep result and the player states, as for the profile pockets.
"""
from __future__ import annotations

from dataclasses import replace
import math

import pyclipper

from . import layers as L
from .milling_opening import FIT_MM, Setup, setup_layers
from .toolpath import Toolpath, ToolpathMove

# A region narrower than this is no roughing work (mm): the mesh chords and the outline cleaning.
MIN_WIDTH_MM = 0.01
# Largest number of paths of one level roughing operation (raster lines and outlines of every level). Each level is
# one layer removal (``LayerReplay.cut_level``), so the paths cost moves, not material work.
MAX_PASSES = 40000
# Largest number of end mills of one level roughing: the largest that reaches, then smaller ones on what it leaves.
MAX_ROUGHING_TOOLS = 3
# Arc tolerance of the roughing offsets and the outline cleaning before them (mm). Roughing needs no sub-micrometre
# arcs: the offsets of large part outlines by the cutter radius took minutes at 0.25 µm (NIST CTC-01). The cutter
# radius gets the tolerance added, so the chords never bring the cutter nearer to the part.
ROUGH_ARC_MM = 0.005
ROUGH_CLEAN_MM = 0.002
# Outline cleaning of the roughing prisms of the B-Rep result (mm): one Boolean of a few hundred edges, not thousands.
BREP_CLEAN_MM = 0.01


def filled(paths):
    """The outer loops only: holes and closed pockets inside the outline are part of the region."""
    outer = [path for path in paths or [] if pyclipper.Orientation(path)]
    return L.union(outer) if outer else []


def _offset(paths, distance_mm: float):
    if not paths:
        return []
    clean = [path for path in pyclipper.CleanPolygons(paths, ROUGH_CLEAN_MM * L.SCALE) if len(path) >= 3]
    job = pyclipper.PyclipperOffset(2.0, ROUGH_ARC_MM * L.SCALE)
    job.AddPaths(clean, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
    return job.Execute(distance_mm * L.SCALE)


def closing(paths, radius: float):
    """What a disk of the radius cannot reach outside the paths (grow, then shrink), on the safe side of the arcs."""
    grow = radius + FIT_MM + ROUGH_ARC_MM + ROUGH_CLEAN_MM
    return _offset(_offset(paths, grow), -radius) if paths else []


class LevelPlan:
    """The roughing bands of one setup: (z_low, z_high, region, shadow) in the setup frame, top down."""

    def __init__(self, layers, bands, silhouette):
        self.layers = layers
        self.bands = bands
        self.silhouette = silhouette

    @property
    def face(self) -> float:
        return self.layers.face

    def deepest(self) -> float:
        """The depth below the setup face of the lowest band floor with material (0: no work)."""
        return max((self.face - band[0] for band in self.bands), default=0.0)

    def volume(self) -> float:
        return sum(L.area(region) * (high - low) for low, high, region, _ in self.bands)

    def band_reach(self, band, envelope_mm: float) -> float:
        """The reach that a band needs (Rajesh, 2026-10-26): from the highest material inside the band region grown by
        the holder radius and the clearance, down to the band floor. The bands above it are cut first (top down), so
        their regions are gone; taller material farther away has no effect."""
        low, high, region, _ = band
        cache = self.__dict__.setdefault("_reach", {})
        key = (round(low, 6), round(envelope_mm, 3))
        if key in cache:
            return cache[key]
        envelope = _offset(region, envelope_mm)
        top = high
        layers = self.layers
        for index in reversed(range(layers.count)):
            z_low, z_high = layers.heights(index)
            if z_high <= high + L.Z_MERGE_MM:
                break
            material = layers.stock[index]
            cut = [path for other in self.bands if other[0] <= z_low + L.Z_MERGE_MM and other[1] >= z_high - L.Z_MERGE_MM
                   for path in other[2]]
            if cut:
                material = L.difference(material, L.union(cut))
            if material and L.area(L.intersection(material, envelope)) > MIN_WIDTH_MM ** 2:
                top = z_high
                break
        cache[key] = top - low
        return cache[key]

    def reach_for(self, tool, depth: float | None = None) -> float:
        """The reach that the tool needs for the bands down to ``depth`` below the setup face (default: all)."""
        from .tool_selection import HOLDER_CLEARANCE_MM, holder_radius
        envelope = holder_radius(tool) + HOLDER_CLEARANCE_MM
        bands = [band for band in self.bands if depth is None or self.face - band[0] <= depth + 1e-6]
        return max((self.band_reach(band, envelope) for band in bands), default=0.0)


def keep_paths(profiles, face_selector: str):
    """The regions of the setup frame that the roughing leaves for their own operation (open pockets and slots):
    profiles in the face coordinates of the profile pockets (``run_profile_pocket``)."""
    import cadquery as cq
    from .profile_milling import _profile_wire, _setup_plane
    _, mirror = _setup_plane(face_selector)
    paths = []
    for profile in profiles or ():
        wire, _ = _profile_wire(profile)
        if mirror < 0:
            wire = wire.mirror("XZ")
        paths.extend(L.face_paths(cq.Face.makeFromWires(wire)))
    return L.union(paths) if paths else []


def roughing_layers(stock_shape, part_shape, face_selector: str, *, reach: float, divide: bool = True):
    """The setup layers of the incoming material and the part (``milling_opening.setup_layers``), down to the reach."""
    from .material_ledger import ATTRIBUTE
    ledger = getattr(stock_shape, ATTRIBUTE, None)
    stock_material = None
    if ledger is not None:
        import numpy as np
        setup = Setup.of(face_selector)
        stock_mesh = ledger.mesh((np.eye(3, dtype=np.int64), np.zeros(3)))
        frame = (setup.rotation, np.zeros(3))
        heights = setup.to_setup(stock_mesh.vertices)[:, 2]
        stock_material = ledger.layered(frame, float(heights.min()), float(heights.max()))
    else:
        stock_mesh = L.SolidMesh.of(stock_shape.val() if hasattr(stock_shape, 'val') else stock_shape)
    part_mesh = L.SolidMesh.of(part_shape)
    return setup_layers(stock_mesh, part_mesh, Setup.of(face_selector), reach_depths=[reach],
                        divide_depth=reach if divide else 0.0, stock_material=stock_material)


def level_plan(stock_shape, part_shape, face_selector: str, *, radius: float, reach: float, keep=(),
               divide: bool = True, layers=None, previous=()) -> LevelPlan:
    """The bands that a flat cutter of the radius roughs from the setup face down to the reach.

    ``stock_shape``: a CadQuery workplane with its material ledger, or a shape without a ledger. ``part_shape``:
    the finished part shape. Both use the part frame.
    ``keep``: profiles that the roughing leaves for their own operations (``keep_paths``); they count as part.
    ``divide``: divide the layers that sloped or curved faces cross (the exact shadow); False is a quick probe of the
    planar levels only. ``layers``: the layers of an earlier plan of the same material (no new mesh and sections).
    ``previous``: the plans of the larger cutters of a rest roughing; each band leaves what they cut over its heights."""
    if layers is None:
        layers = roughing_layers(stock_shape, part_shape, face_selector, reach=reach, divide=divide)
    # All part sections, also those below the reach (the shadow skips them).
    # The silhouette has no holes. Remove them before the union, and avoid repeating
    # unchanged outlines at the many height limits of earlier ledger cuts.
    silhouette, previous_outline = [], None
    for section in layers.part:
        outline = filled(section)
        if outline != previous_outline:
            silhouette = L.union(silhouette, outline)
            previous_outline = outline
    silhouette = filled(silhouette)
    kept = keep_paths(keep, face_selector)
    floor = layers.face - reach
    planar = sorted(set(round(value, 7) for value in layers.planar))
    bands = []
    for low, high in zip(planar, planar[1:]):
        if high <= floor + L.Z_MERGE_MM or low >= layers.face - L.Z_MERGE_MM:
            continue
        low, high = max(low, floor), min(high, layers.face)
        index = layers.layer_at(low + L.Z_MERGE_MM * 10)
        shadow = filled(L.union(layers.shadow[index], kept) if kept else layers.shadow[index])
        region = L.difference(L.intersection(layers.stock[index], silhouette), closing(shadow, radius))
        cut = [path for plan in previous for p_low, p_high, p_region, _ in plan.bands
               if p_low <= low + L.Z_MERGE_MM and p_high >= high - L.Z_MERGE_MM for path in p_region]
        if cut and region:
            region = L.difference(region, L.union(cut))
        region = L.opened(region, MIN_WIDTH_MM)
        if region and L.area(region) > MIN_WIDTH_MM ** 2:
            bands.append((low, high, region, shadow))
    bands.sort(key=lambda band: -band[1])
    merged = []
    for band in bands:
        if merged and abs(merged[-1][0] - band[1]) <= L.Z_MERGE_MM and _same(merged[-1][2], band[2]):
            # The same region in two bands (another level of the part, not of this region): one band, fewer passes.
            merged[-1] = (band[0], merged[-1][1], band[2], band[3])
        else:
            merged.append(band)
    return LevelPlan(layers, merged, silhouette)


def _same(first, second) -> bool:
    changed = L.union(L.difference(first, second), L.difference(second, first))
    return L.area(L.opened(changed, MIN_WIDTH_MM)) <= MIN_WIDTH_MM ** 2


def _centres(region, shadow, radius: float):
    """The cutter centres that cut the region without touching the part above the level."""
    if not shadow:
        return _offset(region, radius)
    return L.difference(_offset(region, radius), _offset(shadow, radius + FIT_MM + ROUGH_ARC_MM + ROUGH_CLEAN_MM))


def _raster(centres, step_mm: float):
    """The raster lines (start, end in mm) of the centre region, one row every step."""
    box = L.bounds(centres)
    if box is None:
        return []
    x0, y0, x1, y1 = box
    rows = max(1, math.ceil((y1 - y0) / (step_mm * L.SCALE)))
    clipper = pyclipper.Pyclipper()
    clipper.AddPaths(centres, pyclipper.PT_CLIP, True)
    for row in range(rows + 1):
        y = round(y0 + (y1 - y0) * row / rows)
        clipper.AddPath([(x0 - L.SCALE, y), (x1 + L.SCALE, y)], pyclipper.PT_SUBJECT, False)
    tree = clipper.Execute2(pyclipper.CT_INTERSECTION, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO)
    lines = []
    for path in pyclipper.OpenPathsFromPolyTree(tree):
        if len(path) < 2:
            continue
        (ax, ay), (bx, by) = path[0], path[-1]
        lines.append(((ax / L.SCALE, ay / L.SCALE), (bx / L.SCALE, by / L.SCALE)))
    lines.sort(key=lambda line: (line[0][1], min(line[0][0], line[1][0])))
    # Alternate the direction of the rows (zigzag order).
    return [line if index % 2 == 0 else (line[1], line[0]) for index, line in enumerate(lines)]


def _path(points, z, safe, tool, feed, rpm, kind):
    path = Toolpath("level_rough", safe_z=safe, tool_type=tool.tool_type, tool_diameter_mm=tool.diameter,
                    feed_mm_min=feed, spindle_rpm=rpm,
                    metadata={"coordinate_system": "part_mm", "tool_axis": [0, 0, 1], "pass_role": "roughing", "path_kind": kind})
    x, y = points[0]
    path.append(ToolpathMove("rapid", (x, y, safe)))
    path.append(ToolpathMove("plunge", (x, y, z), feed=feed, spindle_rpm=rpm))
    path.moves.extend(ToolpathMove("feed", (px, py, z), feed=feed, spindle_rpm=rpm) for px, py in points[1:])
    x, y = points[-1]
    path.append(ToolpathMove("retract", (x, y, safe)))
    return path


def level_paths(plan: LevelPlan, tool, *, stepdown: float, stepover: float, feed: float, rpm, safe: float):
    """The horizontal passes of every band: each stepdown level from the band top to its floor, raster then outline."""
    paths = []
    radius = tool.diameter / 2
    for low, high, region, shadow in plan.bands:
        centres = _centres(region, shadow, radius)
        if not centres:
            continue
        count = max(1, math.ceil((high - low) / stepdown - 1e-9))
        outlines = [[(x / L.SCALE, y / L.SCALE) for x, y in loop] for loop in centres if len(loop) >= 3]
        lines = _raster(centres, stepover)
        for level in range(1, count + 1):
            z = high - (high - low) * level / count
            for start, end in lines:
                paths.append(_path([start, end], z, safe, tool, feed, rpm, "raster"))
            for loop in outlines:
                paths.append(_path(loop + [loop[0]], z, safe, tool, feed, rpm, "outline"))
            if len(paths) > MAX_PASSES:
                raise ValueError("The level roughing exceeds the pass limit. Divide the setup into smaller regions.")
    return paths


def run_level_roughing(shape, part_shape, *, tool, face_selector=">Z", reach=None, stepdown=None, feed=100, rpm=None,
                       keep=(), layers=None, previous=()):
    """Rough the waste above each planar level of the setup. Return (result, toolpath, states, reports, plan).

    ``reach``: the deepest level below the setup face (default: the reach of the tool)."""
    from .material_ledger import MaterialLedger
    from .milling_layers import LayerCheckUnavailable, LayerReplay
    from . import milling_layers
    from .milling_replay import finite, validate_cutter
    from .profile_milling import _setup_plane
    from .tool_selection import tool_reach
    import cadquery as cq

    from .tool_library import WALL_CUTTER_TYPES, envelope_cutter
    if tool is None or tool.tool_type not in WALL_CUTTER_TYPES:
        raise ValueError("Level roughing requires a selected flat end mill or an indexable end mill (90° inserts).")
    radius = finite(tool.diameter, "cutter diameter", positive=True) / 2
    # ``reach``: the deepest level below the setup face; the tool reach is checked locally for each band below.
    reach = finite(reach, "level roughing reach", positive=True) if reach else tool_reach(tool)
    maximum = finite((tool.safe_limits or {}).get("max_stepdown_mm", tool.flute_length), "maximum stepdown", positive=True)
    stepdown = min(finite(stepdown, "level roughing stepdown", positive=True) if stepdown else maximum,
                   maximum, tool.flute_length)
    stepover = min(tool.diameter * 0.4, finite((tool.safe_limits or {}).get("max_stepover_mm", tool.diameter * 0.4),
                                               "maximum stepover", positive=True))
    plane, mirror = _setup_plane(face_selector)
    local = cq.Workplane("XY").newObject([plane.toLocalCoords(shape.val())])
    bounds = local.val().BoundingBox()
    plan = level_plan(shape, part_shape, face_selector, radius=radius, reach=reach, keep=keep, layers=layers,
                      previous=previous)
    from .tool_selection import HOLDER_CLEARANCE_MM, holder_radius
    envelope = holder_radius(tool) + HOLDER_CLEARANCE_MM
    # A band whose local reach is more than the tool reach stays for another cutter.
    plan.bands = [band for band in plan.bands if plan.band_reach(band, envelope) <= tool_reach(tool) + 1e-6]
    safe = bounds.zmax + max(2, radius)
    paths = level_paths(plan, tool, stepdown=stepdown, stepover=stepover, feed=feed, rpm=rpm, safe=safe)
    normal = list(plane.zDir.toTuple())
    aggregate = Toolpath("level_rough", safe_z=safe, tool_type=tool.tool_type, tool_diameter_mm=tool.diameter,
                         feed_mm_min=feed, spindle_rpm=rpm,
                         metadata={"coordinate_system": "part_mm", "tool_axis": normal, "face_selector": face_selector,
                                   "maximum_stepdown_mm": stepdown, "stepover_mm": stepover,
                                   "deepest_level_mm": round(plan.deepest(), 6), "band_count": len(plan.bands),
                                   "planned_volume_mm3": round(plan.volume(), 3)})
    if not paths:
        # No waste above a level within the reach: no moves, no time; the material stays as it is.
        aggregate.metadata.update(material_passes=[], no_work=True)
        state = {"shape": shape, "pass_index": 0, "pass_role": "roughing", "label": "No level roughing needed",
                 "geometry_replay": {"no_work": True}}
        return shape, aggregate, [state], [], plan
    validate_cutter(tool, cutting_depth=plan.reach_for(tool), cut_length=stepdown)
    ledger = MaterialLedger.of(shape)
    cross_setup = any(abs(sum(float(cut.frame[0][axis, 2]) * normal[axis] for axis in range(3))) < 1 - 1e-9
                      for cut in ledger.cuts)
    try:
        # An insert cutter sweeps its envelope: the inserts and the body that follows them down (tool_library).
        replay = LayerReplay(local, bounds.zmax, [path.moves[1].position[2] for path in paths], envelope_cutter(tool),
                             ledger=ledger, plane=plane, layer_brep=MaterialLedger.layer_brep(shape)
                             or (not milling_layers.PLAYER_BREP_STATES and cross_setup))
    except LayerCheckUnavailable as error:
        raise ValueError(f"Level roughing needs the layer model of the material: {error}") from error
    # The checks use the layers; the B-Rep result is for the display and the next B-Rep operations.
    replay.prism_clean_mm = BREP_CLEAN_MM
    # One pass for each cutting level (top down): its raster lines and outlines are one layer removal.
    levels: dict[float, list] = {}
    for path in paths:
        levels.setdefault(round(path.moves[1].position[2], 9), []).append(path)
    reports = []
    for index, (z, group) in enumerate(sorted(levels.items(), key=lambda item: -item[0]), 1):
        report = replay.cut_level(group, "selected_cutter_level_sweeps")
        start = len(aggregate.moves)
        for path in group:
            aggregate.moves.extend(replace(move, position=plane.toWorldCoords(move.position).toTuple()) for move in path.moves)
        report.update(pass_index=index, pass_role="roughing", path_kind="level", level_mm=z,
                      move_start_index=start, move_end_index=len(aggregate.moves), tool_axis=normal)
        reports.append(report)
    paths = [group for _, group in sorted(levels.items(), key=lambda item: -item[0])]
    if sum(report["removed_volume_mm3"] for report in reports) <= 1e-8:
        aggregate.metadata.update(material_passes=reports, no_work=True)
        state = {"shape": shape, "pass_index": 0, "pass_role": "roughing", "label": "No level roughing needed",
                 "geometry_replay": {"no_work": True}}
        return shape, aggregate, [state], reports, plan
    result = local.newObject([replay.result_shape()])
    final = result.val()
    from .layer_player import plane_frame
    frame = plane_frame(face_selector, plane)
    states = []
    for index, _ in enumerate(paths):
        local_state, layers = replay.state(final, index, frame)
        world = None if local_state is None else shape.newObject([local_state.transformShape(plane.rG)])
        states.append({"shape": world, "pass_index": index + 1, "pass_role": "roughing",
                       "label": f"After level roughing level {index + 1}", "geometry_replay": reports[index],
                       "player_layers": layers})
    reports[-1]["material_brep"] = replay.brep_source
    if replay.brep_check:
        reports[-1]["material_brep_check"] = {key: round(value, 6) for key, value in replay.brep_check.items()}
    replay.attach_ledger(states[-1]["shape"])
    aggregate.metadata.update(material_passes=reports, removed_volume_mm3=round(replay.material_removed(), 3))
    return states[-1]["shape"], aggregate, states, reports, plan


from dataclasses import dataclass, field
from typing import Any, Optional

from .operations import MachiningOperation, _feed_rate, _resolve_fs_tool_type, _spindle_rpm, _tool_dict, _tool_selection_dict
from .material import get_feeds_speeds
from .tool_library import ToolSpec


@dataclass
class LevelRoughOp(MachiningOperation):
    """Rough the waste above each planar level of one setup (``run_level_roughing``)."""

    part_shape: Any = field(default=None, repr=False)
    keep: tuple = ()
    """Profiles that the roughing leaves for their own operations (open pockets and slots)."""
    layers: Any = field(default=None, repr=False)
    """The setup layers of the material before the first roughing cutter (shared by the rest cutters)."""
    previous: tuple = field(default=(), repr=False)
    """The plans of the larger cutters before this one (rest roughing)."""
    reach_depth: Optional[float] = None
    """The local reach that the selected tool needs (``LevelPlan.reach_for``); None: the tool reach."""
    reach: float = 0.0
    """The deepest level below the setup face that this operation roughs (mm)."""
    stepdown: Optional[float] = None
    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            raise ValueError("Level roughing requires a selected flat end mill (Stock.rough_levels selects it).")
        self.feeds_speeds = get_feeds_speeds(_resolve_fs_tool_type(self.tool.tool_type), self.tool.diameter, self.material)
        # The inventory check of Stock._apply_op: the local reach needs the tool reach, one stepdown needs the flutes.
        from .tool_selection import tool_reach
        self.depth = self.reach_depth if self.reach_depth is not None else min(self.reach or tool_reach(self.tool), tool_reach(self.tool))
        self.start_depth = max(self.depth - min(self.stepdown or self.tool.flute_length, self.tool.flute_length), 0.0)

    def apply(self, shape):
        result, self._level_toolpath, self._material_pass_states, self._pass_reports, plan = run_level_roughing(
            shape, self.part_shape, tool=self.tool, face_selector=self.face_selector, reach=self.reach or None,
            stepdown=self.stepdown, feed=_feed_rate(self.feeds_speeds), rpm=_spindle_rpm(self.feeds_speeds),
            keep=self.keep, layers=self.layers, previous=self.previous)
        self._plan = plan
        self._bands = [{"z_low_mm": round(low, 6), "z_high_mm": round(high, 6), "area_mm2": round(L.area(region), 3)}
                       for low, high, region, _ in plan.bands]
        self._level_toolpath.metadata.update(tool=_tool_dict(self.tool), tool_selection=_tool_selection_dict(self.tool_selection))
        return result

    def to_dict(self) -> dict:
        metadata = getattr(self, "_level_toolpath", None).metadata if hasattr(self, "_level_toolpath") else {}
        return {
            "operation": "level_rough",
            "strategy": "level_roughing",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": self.reach,
            "deepest_level_mm": metadata.get("deepest_level_mm"),
            "maximum_stepdown_mm": metadata.get("maximum_stepdown_mm"),
            "bands": getattr(self, "_bands", []),
            "removed_volume_mm3": metadata.get("removed_volume_mm3", 0.0),
            "no_work": bool(metadata.get("no_work")),
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds if "error" not in self.feeds_speeds else None,
            "material_passes": getattr(self, "_pass_reports", []),
            "geometry_fidelity": "layers_2_5d", "geometry_status": "requires_review",
            "verification_status": "not_verified", "manufacturing_ready": False,
            "review_reasons": ["Level roughing removes the waste above each planar level with the selected end mill. "
                               "Entry moves, chip load and fixture clearance require verification."],
        }

    def to_toolpath(self) -> Toolpath:
        if not hasattr(self, "_level_toolpath"):
            raise ValueError("Level roughing has no retained cutter path. Apply the operation to its incoming material first.")
        return self._level_toolpath


def select_roughing_tool(shape, part_shape, face_selector: str, *, max_depth: Optional[float] = None, keep=()):
    """(tool, selection, depth): the largest available flat end mill that reaches the deepest level with waste, else the
    one that reaches the deepest band it can; depth 0 when the setup has no waste above a level."""
    from .tool_selection import ToolRequirement, ToolUnavailableError, select_tool
    bounds = shape.val().BoundingBox()
    full = max_depth or max(bounds.xlen, bounds.ylen, bounds.zlen)
    probe = level_plan(shape, part_shape, face_selector, radius=0.5, reach=full, keep=keep, divide=False)
    if not probe.bands:
        # The quick probe takes one section for each layer between two planar levels. Waste that is only under a cone
        # or a curved shoulder of that layer is not in it (d2o#1322: the spanner flats of a turned shaft below its
        # Ø10-Ø8 cone). The divided layers see it; they cost more, so only when the quick probe finds no waste.
        probe = level_plan(shape, part_shape, face_selector, radius=0.5, reach=full, keep=keep, divide=True)
        probe.divided = True
    depths = sorted({round(probe.face - band[0], 6) for band in probe.bands}, reverse=True)
    if not depths:
        return None, None, 0.0
    last = None
    for depth in depths:
        try:
            selection = select_tool(ToolRequirement("flat_endmill", min_reach_mm=depth, prefer_largest=True,
                                                    alternative_tool_types=("indexable_end_mill",),
                                                    local_reach=lambda tool, depth=depth: probe.reach_for(tool, depth),
                                                    feature="level_rough",
                                                    reason="the end mill must reach the deepest level with waste above it"))
        except ValueError:
            # A caller selection policy that refuses every candidate (a reach it does not trust): a higher level.
            continue
        if selection.ok:
            selection.probe = probe
            return selection.selected_tool, selection, depth
        last = selection
    if last is None:
        raise ValueError("No available end mill reaches a level with waste above it.")
    raise ToolUnavailableError(last)
