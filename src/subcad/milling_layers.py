"""Flat end mill pass replay on 2.5D material layers (one setup, tool axis +Z of the setup frame).

Each pass of a profile pocket, slot, facing, outside profile or contour cuts at one tip height.
Its sweep is the 2D Minkowski sum of its tool-centre polyline and the cutter disk. The pass removes
that region from every layer between the tip and the flute top. The removed volume of each pass
and the operation region comparison come from 2D areas and layer heights (``layers.py``).

The material result of the operation for the next operation is one B-Rep: the incoming solid less
one prism for each cutting level (the union of that level's sweeps). This is one Boolean for the
operation; the per-pass Booleans and the four comparison Booleans of the B-Rep replay are gone.
The material state after each pass (for the operation player) is the operation result plus the
layer material that a later pass removes, as extruded layer polygons (no Boolean).
"""
from __future__ import annotations

import math

from . import layers as L


# The operation player gets a B-Rep material state after each pass (True, the old player) or only the 2D removed
# region of each pass (False: ``state["player_layers"]``, schema ``layer_player.SCHEMA``; the last pass keeps its
# B-Rep result for the next operation). The B-Rep states are most of the time of a small operation.
PLAYER_BREP_STATES = True


class LayerCheckUnavailable(ValueError):
    """The layer model cannot represent this material (for example an open mesh)."""


def path_sweep(path, tool):
    """(tip height, sweep paths, report) of one horizontal flat-cutter path."""
    cutting = [move for move in path.moves if move.move_type in {"plunge", "feed"}]
    if not cutting:
        raise ValueError("The path has no cutting moves.")
    z = cutting[0].position[2]
    if any(abs(move.position[2] - z) > 1e-9 for move in cutting):
        raise ValueError("The layer replay requires horizontal cutting paths.")
    points = [move.position for move in cutting]
    closed = len(points) > 3 and math.dist(points[0], points[-1]) < 1e-8
    sweep = L.disk_sweep(points, tool.radius, closed=closed)
    return z, sweep, {"exact_segments": len(cutting), "closed_path": closed, "points": points}


def _report(method, segments):
    return {"method": method, "coordinate_system": "part_mm", "geometry_fidelity": "exact_cutter_sweep",
            "material_model": "layers_2_5d", "exact_segments": segments, "sample_interval_mm": None,
            "sampled_poses": 0, "topology_repaired": False, "topology_cleaned": False,
            "arc_tolerance_mm": L.ARC_TOLERANCE_MM, "manufacturing_ready": False, "verification_status": "not_verified"}


class LayerReplay:
    """The replay of the horizontal passes of one operation in its setup frame."""

    def __init__(self, local_shape, z_top: float, levels, tool, *, ledger=None, plane=None, layer_brep=False):
        """``ledger`` (``material_ledger.MaterialLedger``) and ``plane`` (the setup plane of ``local_shape``): the
        layers come from the ledger, not from a mesh of ``local_shape``, and the operation adds its cuts to it.
        ``layer_brep``: ``local_shape`` is the layer solid of the ledger; the result is the layer solid too."""
        self.local = local_shape
        self.layer_brep = layer_brep and ledger is not None
        self.tool = tool
        self.z_top = z_top
        self.levels = sorted(set(levels))
        self.ledger, self.plane = ledger, plane
        self.frame = None
        self._after = None
        # The source of the B-Rep result: the chained Boolean, or the ledger when that Boolean fails.
        self.brep_source = None
        # Outline cleaning of the B-Rep prisms (mm), with no exact arcs: an operation whose checks use only the layers
        # (level roughing of a large part) needs no Boolean of thousands of arc edges. None: exact arcs first.
        self.prism_clean_mm = None
        try:
            if ledger is not None:
                from .material_ledger import plane_frame
                self.frame = plane_frame(plane)
                self.material = ledger.layered(self.frame, self.levels[0], z_top, self.levels)
            else:
                self.material = L.LayeredMaterial.of_solid(local_shape.val(), self.levels[0], z_top, self.levels)
        except ValueError as error:
            raise LayerCheckUnavailable(str(error)) from error
        self.before = self.material.copy_material()
        self.snapshots = []
        self.removed: list[list] = []  # (z low, z high, removed region) rows of each pass
        self.sweeps_by_level: dict[float, list] = {}

    def cut(self, path, method: str):
        z, sweep, info = path_sweep(path, self.tool)
        rows = []
        removed = self.material.remove(sweep, z, z + self.tool.flute_length, collect=rows)
        self.removed.append(rows)
        level = self.sweeps_by_level.setdefault(z, ([], []))
        level[0].extend(sweep)
        level[1].extend(info["points"])
        self.snapshots.append(self.material.copy_material())
        report = _report(method, info["exact_segments"])
        report["removed_volume_mm3"] = removed
        report["layer_count"] = len(self.material.layers)
        return report

    def cut_level(self, paths, method: str):
        """All horizontal paths of one cutting level as one cut: the union of their sweeps, one layer removal (level
        roughing has hundreds of raster passes at each level; one removal for each pass took minutes on a large part)."""
        sweeps, points, segments, z = [], [], 0, None
        for path in paths:
            height, sweep, info = path_sweep(path, self.tool)
            if z is not None and abs(height - z) > 1e-9:
                raise ValueError("One cutting level requires paths at one height.")
            z = height
            sweeps.extend(sweep)
            points.extend(info["points"])
            segments += info["exact_segments"]
        region = L.union(sweeps)
        rows = []
        removed = self.material.remove(region, z, z + self.tool.flute_length, collect=rows)
        self.removed.append(rows)
        level = self.sweeps_by_level.setdefault(z, ([], []))
        level[0].extend(region)
        level[1].extend(points)
        self.snapshots.append(self.material.copy_material())
        report = _report(method, segments)
        report["removed_volume_mm3"] = removed
        report["layer_count"] = len(self.material.layers)
        report["path_count"] = len(paths)
        return report

    def repeat(self):
        """A repeated path: no material change; keep the state of this pass."""
        self.snapshots.append(self.material.copy_material())
        self.removed.append([])

    def player_layers(self, index: int, frame: dict) -> dict:
        """The 2D region that pass ``index`` (0-based) removes, as player bands in the setup frame."""
        from .layer_player import SCHEMA, bands
        return {"schema": SCHEMA, "frame": frame, "removed": bands(self.removed[index])}

    def state(self, final, index: int, frame: dict):
        """The material state of pass ``index`` for the player: the B-Rep state (``PLAYER_BREP_STATES``) or
        None, with its 2D removed region in both cases."""
        layers = self.player_layers(index, frame)
        if index == len(self.removed) - 1:
            return final, layers
        return (self.state_shape(final, index) if PLAYER_BREP_STATES else None), layers

    def result_shape(self):
        """The B-Rep result: the incoming solid less the level prisms, deepest level first.

        A level sweep reaches up to the flute top, so a higher level adds only the part of its region that no
        deeper level covers (2D difference). Most operations need one or two Booleans."""
        from .milling_replay import _boolean
        if self.layer_brep and self.ledger is not None and self.sweeps_by_level:
            return self._ledger_result()
        top = self.z_top + 1.0
        radius = self.tool.radius
        levels = []
        covered, centers = [], []
        for z in sorted(self.sweeps_by_level):
            loops, points = self.sweeps_by_level[z]
            region = L.union(loops)
            centers.extend(points)
            # Polygon chords of one arc in two levels leave slivers thinner than the arc tolerance.
            added = L.opened(L.difference(region, covered), 2 * L.ARC_TOLERANCE_MM) if covered else region
            if added:
                levels.append((z, added, L.cutter_arcs(centers, radius)))
            covered = L.union(covered, region)
        if not levels:
            return self.local.val()
        incoming = self.local.val().Volume()
        expected = incoming - self.material_removed()
        if self.prism_clean_mm:
            import pyclipper
            levels = [(z, [path for path in pyclipper.CleanPolygons(added, self.prism_clean_mm * L.SCALE) if len(path) >= 3], arcs)
                      for z, added, arcs in levels]
        # Exact cutter arcs first; plain polygon prisms (chords within the arc tolerance) if the arcs fail.
        attempts = (False,) if self.prism_clean_mm else (True, False)
        for exact in (() if self.layer_brep else attempts):
            result = self.local.val()
            try:
                for z, added, arcs in levels:
                    for tool in L.prisms(added, z, max(top, z + self.tool.flute_length), arcs=arcs if exact else None):
                        result = _boolean(result, [tool], "cut")
            except ValueError:
                continue
            # The B-Rep result and the layer removal must agree (mesh chords and layer midpoints: 0.5 %).
            if result.Solids() and result.isValid() and abs(result.Volume() - expected) <= max(
                    1e-3, 0.005 * max(incoming - expected, 0.0)):
                self.brep_source = "chained_brep"
                return result
        if self.ledger is None:
            raise LayerCheckUnavailable("The B-Rep result and the layer removal do not agree.")
        return self._ledger_result()

    def _ledger_result(self):
        # The incoming B-Rep cannot take this cut (for example coincident walls of two cutter sweeps). The layers
        # are the material; the B-Rep comes from the ledger (material_ledger.MaterialLedger.solid).
        try:
            result = self.ledger_after().solid(self.plane)
        except ValueError as error:
            raise LayerCheckUnavailable(str(error)) from error
        self.brep_source = "layer_prisms"
        self.ledger_after().brep_from_layers = True
        return result

    def ledger_after(self):
        """The ledger with the cuts of this operation: each cutting level's sweep region from the tip to the flute top
        (the layers that ``LayeredMaterial.remove`` changes)."""
        if self._after is None:
            from .material_ledger import Cut
            cuts = [Cut(self.frame, z, z + self.tool.flute_length, L.union(loops))
                    for z, (loops, _) in sorted(self.sweeps_by_level.items())]
            self._after = self.ledger.with_cuts(cuts)
        return self._after

    def attach_ledger(self, workplane):
        """Attach the ledger after this operation to its result workplane (no ledger: nothing)."""
        if self.ledger is not None:
            self.ledger_after().attach(workplane)
        return workplane

    def material_removed(self) -> float:
        return sum((L.area(first) - L.area(second)) * layer.height
                   for first, second, layer in zip(self.before, self.snapshots[-1] if self.snapshots else self.before,
                                                   self.material.layers))

    def state_shape(self, result, index: int):
        """The material after pass ``index`` (operation player): the result plus the layer material that later
        passes remove, as extruded layer polygons. A cut changes the material only at the cutting levels, so
        the layers between two cutting levels show as one prism of their middle layer (display only)."""
        import bisect

        import cadquery as cq
        snapshot = self.snapshots[index]
        final = self.snapshots[-1]
        groups: dict[int, list] = {}
        for layer, material, last in zip(self.material.layers, snapshot, final):
            band = bisect.bisect_right(self.levels, layer.z_low + L.Z_MERGE_MM)
            group = groups.setdefault(band, [layer.z_low, layer.z_high, []])
            group[0], group[1] = min(group[0], layer.z_low), max(group[1], layer.z_high)
            group[2].append((material, last))
        for group in groups.values():
            material, last = group[2][len(group[2]) // 2]
            group[2] = [] if material is last else L.difference(material, last)
        solids, run = [], None
        for group in sorted(groups):
            z_low, z_high, pieces = groups[group]
            extra = L.display_paths(pieces)
            if not extra or L.area(extra) * (z_high - z_low) <= 1e-9:
                continue
            if run is not None and run[2] == extra and abs(run[1] - z_low) <= L.Z_MERGE_MM:
                run[1] = z_high
                continue
            if run is not None:
                solids.extend(L.prisms(run[2], run[0], run[1], single=True))
            run = [z_low, z_high, extra]
        if run is not None:
            solids.extend(L.prisms(run[2], run[0], run[1], single=True))
        if not solids:
            return result
        return cq.Compound.makeCompound([result, *solids])

    def comparison(self, target, z_low: float, z_high: float):
        remaining, overcut = L.region_comparison(self.before, self.material, target, z_low, z_high)
        return {"method": "layered_2d_operation_region", "remaining_target_material_mm3": remaining,
                "overcut_volume_mm3": overcut, "contact_tolerance_mm": L.CONTACT_MM,
                "arc_tolerance_mm": L.ARC_TOLERANCE_MM, "layer_count": len(self.material.layers),
                "tolerance_verified": False, "manufacturing_ready": False}
