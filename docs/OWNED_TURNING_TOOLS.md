# Recorded Turning assemblies

SubCAD 0.2.23 accepts `TurningToolSpec` records through `turning_tool_inventory`.
The caller supplies an immutable inventory revision and physical assembly records.
The context remains separate from Milling cutter selection.
Concurrent operations retain their own inventory contexts.

Each recorded tool retains its insert ISO, holder ISO, role, hand, tip direction, and nose radius.
The physical assembly retains the caller's machine, turret station, offset number, and measurement attribution.
Unknown measurements remain unknown. The record contains no Milling flute dimensions or rotary cutter diameter.

Native facing, outside profiles, outside diameters, boring, grooves, and threads select the corresponding tool role.
The caller can distinguish rough and finish profiles through the context's `role` argument.
Selection rejects unavailable assemblies and explicit tools that differ from the current inventory.
Selection also checks recorded minimum bore, cutting depth, groove width, thread pitch range, and thread profile angle.
An empty inventory raises `ToolUnavailableError` with retained selection reasons.

The operation retains the selected assembly and inventory revision in its process plan.
Tool selection does not qualify insert material suitability, holder clearance, tool compensation, feeds, or spindle synchronization.
The native geometry remains an ideal material preview. Manufacturing verification remains required.
Internal operations retain the separate requirement to check the actual starting bore.
The thread preview retains its generic V profile and thread-standard verification reason.

`test_subcad_owned_turning_tools.py` checks native material removal, retained assignments, empty inventories, incompatible tools, and concurrent contexts.
`test_subcad_turning_geometry.py` checks the existing native Turning geometry.
