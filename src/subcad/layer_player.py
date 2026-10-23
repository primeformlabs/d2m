"""2D layer data for the operation player (schema ``subcad.layer_player.v1``, docs/MILLING_OPENING_VERDICT.md).

The server sends 2D polygons with height ranges. The browser extrudes each one along the setup axis and
places it with the setup frame. The server builds no 3D state for the player.

The material at any moment of the program is

    part ∪ material left at the end ∪ (the regions that later passes remove)

and these sets do not overlap. So the player shows a pass by hiding its removed region: no Boolean in the
browser and none on the server.
"""
from __future__ import annotations

import pyclipper

from . import layers as L

SCHEMA = "subcad.layer_player.v1"
# Integer coordinates of the player data: 1 unit = 1 µm.
UNITS_PER_MM = 1000
# Outline tolerance of the player polygons (mm): vertices that move the outline by less are removed.
DISPLAY_TOLERANCE_MM = 0.002


def encode(paths) -> list[list[int]]:
    """Closed loops as flat integer lists [x0, y0, x1, y1, ...] in µm. Outer loops are counter-clockwise,
    holes clockwise (pyclipper orientation, the even-odd region is the same)."""
    if not paths:
        return []
    step = L.SCALE // UNITS_PER_MM
    loops = []
    for path in pyclipper.CleanPolygons(paths, DISPLAY_TOLERANCE_MM * L.SCALE):
        if len(path) < 3:
            continue
        flat = []
        for x, y in path:
            flat.extend((int(round(x / step)), int(round(y / step))))
        loops.append(flat)
    return loops


def bands(rows) -> list[dict]:
    """[(z low, z high, paths), ...] as bands; consecutive rows with the same loops become one band."""
    result = []
    for low, high, paths in sorted(rows, key=lambda row: row[0]):
        loops = encode(paths)
        if not loops or high - low <= L.Z_MERGE_MM:
            continue
        if result and result[-1]["loops"] == loops and abs(result[-1]["z"][1] - low) <= 1e-6:
            result[-1]["z"][1] = round(high, 6)
            continue
        result.append({"z": [round(low, 6), round(high, 6)], "loops": loops})
    return result


def setup_frame(name: str, matrix, origin=(0.0, 0.0, 0.0)) -> dict:
    """``matrix`` rows: the part x, y, z of the setup x, y, z directions (columns are the setup axes)."""
    return {"name": name, "setup_to_part": [[float(value) for value in row] for row in matrix],
            "origin_mm": [float(value) for value in origin]}


def plane_frame(name: str, plane) -> dict:
    """The setup frame of a CadQuery plane (the frame of the layer replay of an operation)."""
    x, y, z = plane.xDir.toTuple(), plane.yDir.toTuple(), plane.zDir.toTuple()
    return setup_frame(name, [[x[i], y[i], z[i]] for i in range(3)], plane.origin.toTuple())


def removal_bands(before_shape, after_shape, setup, chord: float = 0.01) -> list[dict]:
    """The material that an operation removed, as bands in the setup frame, from two solids (mesh sections only).

    For operations that have no layer replay (drilling, surface milling): the layers come from the horizontal faces
    of both solids and the chord bound of their sloped faces (``milling_opening.setup_layers``)."""
    from . import milling_opening as M
    setup = M.Setup.of(setup)
    before, after = L.SolidMesh.of(before_shape), L.SolidMesh.of(after_shape)
    layers = M.setup_layers(before, after, setup, chord)
    rows = []
    for index in range(layers.count):
        a, b = layers.heights(index)
        rows.append((a, b, L.opened(L.difference(layers.stock[index], layers.part[index]))))
    return bands(rows)


def document(setups: list[dict], passes: list[dict], *, part: dict | None = None, left: list | None = None) -> dict:
    """The player document.

    ``setups``: frames (``setup_frame``). ``passes``: in program order, each {"setup": name, "operation_id",
    "pass_index", "label", "removed": bands}. ``part``: how the browser shows the part (for example the display
    mesh link). ``left``: [{"setup": name, "bands": bands}], the material that stays at the end."""
    return {"schema": SCHEMA, "units": "mm", "coordinate_units_per_mm": UNITS_PER_MM,
            "display_tolerance_mm": DISPLAY_TOLERANCE_MM, "setups": setups, "part": part or {},
            "left": left or [], "passes": passes}
