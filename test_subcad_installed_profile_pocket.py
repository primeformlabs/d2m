"""Run the installed profile pocket engine without source checkout imports."""
import math
from pathlib import Path

import cadquery as cq
import subcad
from subcad import Stock, ToolSpec, tool_inventory

assert 'site-packages' in str(Path(subcad.__file__).resolve())
tool = ToolSpec('flat_endmill', 2, catalog_id='INSTALLED-POCKET', physical_assembly={'owned': True})
stock = Stock.rectangular(12, 12, 8)
with tool_inventory([tool], revision='installed-pocket', strict=True):
    result = stock.profile_pocket({'type': 'rectangle', 'length': 8, 'width': 8}, 1,
        islands=[{'type': 'rectangle', 'length': 2, 'width': 2}])
record = result.process_plan()['operations'][-1]
assert result._shape.val().isInside(cq.Vector(0, 0, 3.5))
assert not result._shape.val().isInside(cq.Vector(3, 0, 3.5))
assert record['operation_target_comparison']['overcut_volume_mm3'] < 1e-6
assert record['operation_target_comparison']['remaining_target_material_mm3'] > 0
assert len(result._state_history) == len(record['material_passes']) + 1
assert record['tool_selection']['inventory_revision'] == 'installed-pocket'
assert {row['pass_role'] for row in record['material_passes']} == {'roughing', 'finishing'}
assert record['achieved_tolerance_estimate_mm'] is None
slot = stock.profile_pocket({'type': 'slot', 'length': 8, 'width': 2}, 1, tool=tool, finish_allowance=0)
assert abs(stock.volume - slot.volume - (12 + math.pi)) < 1e-6
cutout = stock.profile_cutout({'type': 'rectangle', 'length': 6, 'width': 6}, depth=1, tool=tool)
assert abs(stock.volume - cutout.volume - (144 - 36)) < 1e-6
for face, axis, position in [
    ('>Z', [0, 0, 1], lambda u, v, w: (u, v, w)),
    ('<Z', [0, 0, -1], lambda u, v, w: (u, v, -w)),
    ('>X', [1, 0, 0], lambda u, v, w: (w, u, v)),
    ('<X', [-1, 0, 0], lambda u, v, w: (-w, u, v)),
    ('>Y', [0, 1, 0], lambda u, v, w: (u, w, v)),
    ('<Y', [0, -1, 0], lambda u, v, w: (u, -w, v)),
]:
    part = Stock.rectangular(8, 8, 8).profile_pocket({'type': 'rectangle', 'length': 4, 'width': 4}, 1,
        tool=tool, face_selector=face)
    assert not part._shape.val().isInside(cq.Vector(*position(0, 0, 3.5)))
    assert part._shape.val().isInside(cq.Vector(*position(3, 0, 3.5)))
    assert part.process_plan()['operations'][-1]['toolpath']['metadata']['tool_axis'] == axis
print('Installed profile pocket: actual cutter corners, islands, raster/finish states, outside clearing, matching slots, and six setup faces pass.')
