"""Run with python -I after wheel installation, outside the source checkout."""
import math
from pathlib import Path
import tempfile

import cadquery as cq

import subcad
from subcad import Stock, ToolSpec, compare_to_target, get_feeds_speeds, list_materials, to_sim_tool, replay_milling_independently
from simulation.material_removal import MaterialRemovalEngine
from simulation.tri_dexel import TriDexelModel

assert "site-packages" in str(Path(subcad.__file__).resolve())
assert MaterialRemovalEngine and TriDexelModel
assert "aluminum_6061" in list_materials()
assert "error" not in get_feeds_speeds("end_mill", 8, "aluminum_6061")
drill = ToolSpec("drill", 4, tip_angle=118, flute_length=20, catalog_id="ENGINEERING-INSTALLED-DRILL",
    physical_assembly={"owned": True, "assembly_id": "ENGINEERING-INSTALLED-DRILL"})
drilled = Stock.rectangular(8, 8, 6).drill(4, through=True, spot_drill=False, face_selector="<X", tool=drill)
drill_path = drilled.process_plan()["operations"][-1]["toolpath"]
assert drill_path["metadata"]["coordinate_system"] == "stock_mm"
assert drill_path["metadata"]["tool_frame"]["z_axis"] == [-1, 0, 0]
assert drill_path["metadata"]["cad_drill_bottom"] == "through"
assert to_sim_tool(ToolSpec("flat_endmill", 8, flute_length=20)) is not None
comparison_stock = Stock.rectangular(12, 12, 8)
faced = comparison_stock.face_mill(1)
assert faced.process_plan()["operations"][-1]["toolpath"]["moves"]
assert compare_to_target(comparison_stock, comparison_stock, tolerance_mm=0.5)["status"] == "pass"

stock = Stock.rectangular(40, 40, 20, material="aluminum_6061")
result = stock.pocket(width=20, length=20, depth=5, corner_radius=2)
assert abs(stock.volume - result.volume - (400 - (4 - math.pi) * 4) * 5) < 1e-6
assert len(result.process_plan()["operations"]) >= 1
turned = Stock.round_bar(30, 40, axis="X").turn_face(10).turn_od(30, 25).turn_id(10, 10).turn_groove(2, 1, -5)
expected = math.pi * 15 ** 2 * 25 - math.pi * 5 ** 2 * 10 - math.pi * (15 ** 2 - 14 ** 2) * 2
assert abs(turned.volume - expected) < 1e-6
assert turned.process_plan()["operations"][-1]["toolpath"]["metadata"]["process"] == "turn"
assert len(turned._state_history) == 5
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory) / "result.stl"
    result.to_stl(str(path))
    assert path.stat().st_size > 100

curved = Stock.round_bar(40, 20, axis="X").turn_od(18, 40)
cross_drilled = curved.drill(diameter=4, depth=20, through=True, face_selector=">Z", spot_drill=False)
finished = cross_drilled.turn_od(16, 40)
assert finished.volume < math.pi * 8 ** 2 * 40
assert not finished._shape.val().isInside(cq.Vector(0, 0, 0))
plane_stock = Stock.rectangular(8, 8, 6)
plane_tool = ToolSpec("flat_endmill", 4)
plane = plane_stock.surface_mill({"type": "plane", "height_mm": 1}, tool=plane_tool, tolerance_mm=0.3)
assert abs(plane.volume - 256) < 1e-6
assert len(plane._state_history) > 3
from subcad import Toolpath
_, replay = replay_milling_independently(plane_stock,
    Toolpath.from_dict(plane.process_plan()["operations"][-1]["toolpath"]), plane_tool,
    resolution_mm=0.5, target=plane)
assert replay["target_comparison"]["status"] == "sampled_match"
assert replay["manufacturing_ready"] is False
dome_stock = Stock.rectangular(8, 8, 8)
dome_stock._shape = dome_stock._shape.cut(cq.Workplane("XY", origin=(0, 0, -5)).circle(0.5).extrude(10))
dome = dome_stock.dome_mill(2.5, 1, tool=ToolSpec("ball_endmill", 2,
    safe_limits={"max_stepdown_mm": 0.5}), tolerance_mm=0.3)
assert dome.volume < dome_stock.volume
assert not dome._shape.val().isInside(cq.Vector(0, 0, 2))
assert not dome._shape.val().isInside(cq.Vector(3.5, 3.5, 3.6))
assert {row["pass_role"] for row in dome.process_plan()["operations"][-1]["material_passes"]} == {"roughing", "finishing"}
try:
    plane_stock.surface_mill({"id": "missing_patch"})
except ValueError as error:
    assert "explicit geometry" in str(error)
else:
    raise AssertionError("An unresolved surface cannot prove material removal.")
print("Installed runtime: imports, material rules, tool conversion, target comparison, native pockets, Turning states, curved-stock drilling, surface passes, independent Milling replay, operation plans, and STL export pass.")
