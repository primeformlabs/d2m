# Feature depth and tool reach

Version 0.2.28. Module `src/subcad/feature_depth.py`, rule `tool_selection.tool_reach`.

## Two numbers for each cutter

A cutter has two limits. Each limit has its own check.

| Number | What it limits | Value of the tool |
|---|---|---|
| Flute length | The material height that one pass cuts | `ToolSpec.flute_length` |
| Reach | The depth of the tip below the setup face | `tool_reach(tool) = min(stickout_mm, reach_mm or flute_length)` |

The tool moves through the air above the material. So a cut that starts deep below the setup face needs a long
reach, but its flutes cut only the material height. A tool with no recorded reach reaches its flute length, as before.

`ToolRequirement` has `min_flute_length_mm` (the cut) and `min_reach_mm` (the depth). Without `min_reach_mm` the
reach is the flute length: an old caller keeps its old result. A drill or a reamer adds its point length to both.
`validate_inventory_tool(tool, depth=..., cut_length=...)` and `milling_replay.validate_cutter(tool,
cutting_depth=..., cut_length=...)` check the same two numbers.

## Where a round feature starts

`material_span(shape, face, cx, cy, radius)` intersects the incoming material with a cylinder of the feature
radius along the setup axis. It gives two depths below the setup face:

- entry: the top of the material inside the circle;
- far: the bottom of the material inside the circle.

`Stock.drill`, `Stock.bore` and `Stock.ream` use it (`Stock._feature_depths`):

- A through feature goes from the entry to the far depth (the bottom of its material column, not of the stock box).
- A blind feature keeps its depth from the setup face, and starts at the entry when the entry is above its floor.

The operation records `start_depth` (`start_depth_mm` in its record). The drill moves rapid to a clearance above the
entry and cuts from there; the circular-milling passes start at the entry. The time of the cut is the time of the
material height, not of the air above it.

If the Boolean of the probe fails (a ledger layer solid can refuse it), the feature keeps the old rule: the stock box.

## Example

A block 30 mm high has a step: one half is 20 mm lower. A through hole in the low half has its material from 20 mm to
30 mm below the setup face. Its drill needs 10 mm of flute (plus the point) and 30 mm of reach (plus the point). The
old rule needed 30 mm of flute (`test_subcad_feature_depth.py`).
