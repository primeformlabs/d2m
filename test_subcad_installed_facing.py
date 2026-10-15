"""Check native facing from the installed runtime, including bottom datums."""
from pathlib import Path
import cadquery as cq
import subcad
from subcad import Stock, ToolSpec, tool_inventory

assert 'site-packages' in str(Path(subcad.__file__).resolve())
tool = ToolSpec('face_mill', 12, catalog_id='INSTALLED-FACE', physical_assembly={'owned': True})
stock = Stock.rectangular(12, 10, 12)
with tool_inventory([tool], revision='installed-facing', strict=True):
    first = stock.face_mill(2)
    final = first.face_mill(2, face_selector='<Z')
assert abs(final.volume - 960) < 1e-5
assert abs(final.bounding_box['z_min'] + 4) < 1e-6
assert abs(final.bounding_box['z_max'] - 4) < 1e-6
for operation in final.process_plan()['operations']:
    assert operation['material_passes']
    assert operation['tool']['catalog_id'] == 'INSTALLED-FACE'
    assert operation['geometry_fidelity'] == 'nominal_face_mill_envelope'
    assert operation['achieved_tolerance_estimate_mm'] is None
assert final.process_plan()['operations'][-1]['toolpath']['metadata']['tool_axis'] == [0, 0, -1]
print('Installed facing: selected face-mill envelope, native retained passes, owned identity, and both finished-part datums pass.')
