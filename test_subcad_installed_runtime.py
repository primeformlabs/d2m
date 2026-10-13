"""Run with python -I after wheel installation, outside the source checkout."""
import math
from pathlib import Path
import tempfile

import subcad
from subcad import Stock, ToolSpec, compare_to_target, get_feeds_speeds, list_materials, to_sim_tool
from simulation.material_removal import MaterialRemovalEngine
from simulation.tri_dexel import TriDexelModel

assert "site-packages" in str(Path(subcad.__file__).resolve())
assert MaterialRemovalEngine and TriDexelModel
assert "aluminum_6061" in list_materials()
assert "error" not in get_feeds_speeds("end_mill", 8, "aluminum_6061")
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

print("Installed runtime: imports, material rules, tool conversion, target comparison, native pockets, Turning states, operation plans, and STL export pass.")
