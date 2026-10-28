# Level roughing

Version 0.2.30. Module `src/subcad/level_roughing.py`, operation `LevelRoughOp`, method `Stock.rough_levels`.

## Why

A stepped part has its features on several planar levels. A hole in a lower section can be drilled from its own
level only when the waste above that level is gone. Before level roughing, each feature cut started at the top of
the stock box, so its tool needed the full stock height of flute.

## The region of each level

For one setup (one fixed tool axis) and a flat end mill of radius r:

| Name | Meaning |
|---|---|
| silhouette | The part outline along the tool axis: the union of all part sections, holes filled |
| shadow(z) | The part sections at and above the height z, holes and closed pockets filled |
| region(z) | The incoming material ∩ silhouette − closing(shadow(z), r) |

- The closing (`milling_opening`) is what a disk of radius r cannot reach without touching the part at or above z.
  So the cutter never cuts the part and never goes under an overhang.
- The silhouette keeps the outside profile for its own operation.
- The filled shadow keeps the holes and the closed pockets for their own operations.
- The shadow grows downward. So the region at the bottom of a band (two planar heights) is safe for the whole band.
  Two bands with the same region become one band.

The incoming material comes from the material ledger throughout tool selection and replay.
The ledger supplies the stock sections and their height limits, including cuts from other setups.
The target part still supplies its own sections.
The silhouette combines filled outer outlines once for each change of outline.
Repeated sections from earlier cuts do not enter one large polygon union.

## Passes

At each stepdown level of each band, from the band top to its floor: a raster of the free cutter centres (stepover
0.4 × diameter, or the tool's `max_stepover_mm`), then the outline of the centres. The stepdown is the tool's
`max_stepdown_mm`, else its flute length. The layer replay (`milling_layers.LayerReplay`) removes the material,
writes the material ledger and makes the B-Rep result and the player states, as for a profile pocket.

In a layer-player preview, roughing after cuts from another setup makes its display solid directly from the ledger.
It does not calculate display Booleans or cutter arcs for that solid.
The material checks still use the ledger layers.
Program mode and the default B-Rep player keep their existing result path.

## Tool

Without a tool, `select_roughing_tool` probes the levels with a 0.5 mm radius and takes the largest available flat
end mill that reaches the deepest level with waste. If no tool reaches it, the next higher level is tried. The
operation records its depth (the reach) and one stepdown as the flute length for the inventory check.

A setup with no waste above a level gives no operation: `Stock.rough_levels` returns the same stock.

## Test

`test_subcad_level_roughing.py`: a block with a 20 mm step, a closed pocket and a hole. The roughing removes the
step waste only; a drill in the low section then starts 20 mm below the setup face.
