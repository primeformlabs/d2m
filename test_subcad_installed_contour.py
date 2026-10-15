"""Run with python -I after wheel installation, outside the source checkout."""
from pathlib import Path

import cadquery as cq
import subcad
from subcad import Stock, ToolSpec, tool_inventory, replay_milling_independently
from subcad.toolpath import Toolpath

assert "site-packages" in str(Path(subcad.__file__).resolve())
tool = ToolSpec("flat_endmill", 2, catalog_id="INSTALLED-CONTOUR", physical_assembly={"owned": True})
stock = Stock.rectangular(12, 12, 6)
with tool_inventory([tool], revision="installed-contour", strict=True):
    outside = stock.profile_contour({"type": "rectangle", "length": 6, "width": 6}, 2)
    inside = stock.profile_contour({"type": "rectangle", "length": 6, "width": 6}, 2, side="inside")
assert outside._shape.val().isInside(cq.Vector(0, 0, 2))
assert not outside._shape.val().isInside(cq.Vector(4, 0, 2))
assert not inside._shape.val().isInside(cq.Vector(2, 0, 2))
assert inside._shape.val().isInside(cq.Vector(0, 0, 2))
record = inside.process_plan()["operations"][-1]
assert record["tool_selection"]["inventory_revision"] == "installed-contour"
assert len(inside._state_history) == len(record["material_passes"]) + 1
assert {row["pass_role"] for row in record["material_passes"]} == {"roughing", "finishing"}
assert record["achieved_tolerance_estimate_mm"] is None
path = Toolpath.from_dict(record["toolpath"])
model, report = replay_milling_independently(stock, path, tool, resolution_mm=0.5, target=inside)
assert not model.get_material_at((2, 0, 2))
assert model.get_material_at((0, 0, 2))
assert report["target_comparison"]["samples"] > 0
assert not report["target_comparison"]["tolerance_verified"]
assert not report["manufacturing_ready"]

for face, axis, point in [
    (">Z", [0, 0, 1], lambda u, v, w: (u, v, w)),
    ("<Z", [0, 0, -1], lambda u, v, w: (u, v, -w)),
    (">X", [1, 0, 0], lambda u, v, w: (w, u, v)),
    ("<X", [-1, 0, 0], lambda u, v, w: (-w, u, v)),
    (">Y", [0, 1, 0], lambda u, v, w: (u, w, v)),
    ("<Y", [0, -1, 0], lambda u, v, w: (u, -w, v)),
]:
    result = Stock.rectangular(10, 10, 10).profile_contour(
        {"type": "rectangle", "length": 4, "width": 4}, 1, tool=tool, face_selector=face)
    assert result._shape.val().isInside(cq.Vector(*point(0, 0, 4.5)))
    assert not result._shape.val().isInside(cq.Vector(*point(3, 0, 4.5)))
    path = Toolpath.from_dict(result.process_plan()["operations"][-1]["toolpath"])
    assert path.metadata["tool_axis"] == axis
    model, replay = replay_milling_independently(Stock.rectangular(10, 10, 10), path, tool, resolution_mm=1)
    assert model.get_material_at(point(0, 0, 4.5))
    assert not model.get_material_at(point(3, 0, 4.5))
    assert replay["tool_axis"] == axis
print("Installed contour runtime: compensated inside/outside channels, retained passes, owned tools, and six-axis native/independent replay pass.")
