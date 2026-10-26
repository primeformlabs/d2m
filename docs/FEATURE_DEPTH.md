# Feature depth and tool reach

Version 0.2.28; local reach 0.2.29. Module `src/subcad/feature_depth.py`, rule `tool_selection.tool_reach`.

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

## Local reach (0.2.29, Rajesh 2026-10-26)

The reach is not measured from the setup face (the top-most point of the part or the stock). That point can be far
from the feature and does not limit the tool there. The reach that a tool needs is measured locally:

    reach = deepest cut depth − depth of the highest material inside the envelope

The envelope is the region that the tool and its holder take above the feature: the cutter path grown by the holder
radius and `HOLDER_CLEARANCE_MM` (2 mm). The holder is `physical_assembly.holder_diameter_mm`, else the holder that D2O
assumes (`ASSUMED_HOLDER_DIAMETER_MM`, 50 mm), never smaller than the cutter. The material is the incoming material of
the operation (after level roughing). Taller material farther away has no effect.

- Drill and reamer: the envelope is a circle round the hole axis (`feature_depth.LocalReach`, path "axial").
- Circular milling: the hole circle less the cutter radius, grown by the holder radius (path "circle").
- Level roughing: each band region grown by the holder radius; the bands above it are already cut
  (`LevelPlan.band_reach`). A band that the tool does not reach locally stays for another cutter.

`ToolRequirement.local_reach(tool)` gives the reach for each candidate (its holder sets the envelope); `select_tool`,
the inventory check (`reach_depth`) and the replay checks use it. `min_reach_mm` stays the depth from the setup face
for the records.
