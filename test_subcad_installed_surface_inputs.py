"""Supplemental installed-runtime checks for surface depth and cutter inputs."""
import math
from pathlib import Path

import subcad
from subcad import Stock, ToolSpec, tool_inventory

assert "site-packages" in str(Path(subcad.__file__).resolve())
tool = ToolSpec("flat_endmill", 1, flute_length=0.5, stickout_mm=2, reach_mm=0.5,
    catalog_id="INSTALLED-SHALLOW", physical_assembly={"owned": True})
with tool_inventory([tool], strict=True):
    result = Stock.rectangular(4, 4, 4).surface_mill({"type": "plane", "height_mm": 1}, tolerance_mm=0.3)
assert abs(result.volume - 48) < 1e-6
assert abs(result.process_plan()["operations"][-1]["depth_mm"] - 1) < 1e-6
for invalid, reason in [(ToolSpec("flat_endmill", math.nan), "cutter diameter"),
                        (ToolSpec("flat_endmill", 2, flute_length=0), "flute length")]:
    try:
        Stock.rectangular(8, 8, 6).surface_mill({"type": "plane", "height_mm": 1}, tool=invalid)
    except ValueError as error:
        assert reason in str(error)
    else:
        raise AssertionError("An invalid cutter cannot generate a surface path.")
print("Installed surface inputs: owned shallow passes, actual depth, and specific cutter errors pass.")
