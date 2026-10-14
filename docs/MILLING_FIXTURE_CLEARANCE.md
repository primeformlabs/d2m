# Milling fixture clearance

SubCAD 0.2.7 checks continuous linear travel on six cardinal cutter axes.
The cutter axis points from the tip towards the holder.
Toolpaths and fixture zones use `part_mm` coordinates.

The check clips each travel segment against the component's axial overlap.
It calculates the minimum radial distance to each fixture box over that interval.
This detects thin fixture zones between the previous sampled positions.
It also avoids a collision from radial and axial overlaps at different travel times.
Clearance margins expand the radial envelope and axial limits.

Each setup retains its complete fixture specification.
The last fixture cannot replace an earlier setup's fixture.
An empty setup zone list overrides the global zone list.
Duplicate setup names require a new name for the revised setup.

The check uses declared cutter, shank, and holder envelopes.
An intersection means a possible collision with that envelope.
It does not prove an exact collision with a CAD fixture model.
Missing assembly dimensions leave complete clearance unverified.
Invalid bounds, missing paths, unsupported axes, and arcs produce specific reasons.
Arcs cannot become straight chords in this check.
The first retained pose does not prove an unknown approach path.
Retained passes can supply `metadata.pass_entry_position` to check the entry segment.

The structured issues retain setup geometry, fixture identity, tool identity, cutter axis, component, move index, and overlap position.
Every clearance issue sets `manufacturing_ready=false`.
No intersection does not prove clamping force, access, machine travel, stock fit, or manufacturing readiness.
The existing stock-fit checks use declared length and width.
They do not verify orientation-dependent gripping or contact stability.

`ProcessPlan.validation_buckets` preserves error severity.
A validation exception produces an error instead of an empty report.
Both fixture-check entry points use the same continuous check.

Focused checks: `python -m unittest test_subcad_fixture_clearance -q`.
Existing integration check: `python test_subcad_manufacturing_trust_validation.py`.
