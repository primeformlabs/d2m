# Milling verdict by the morphological opening

SubCAD 0.2.25 gives the milling review verdict with an exact 2D formula: `milling_opening.part_verdict`.
The verdict does not replay the passes. The passes stay for the program time and the NC program only.
The operation player gets 2D layer data (`layer_player.py`). The server builds no 3D state for the player.

## Verdict

### One setup, one layer, one flat cutter

A 3-axis setup has one tool axis (+Z of the setup frame). For each layer of the setup:

* `R` = stock section − part section (the material to remove);
* `shadow` = the union of the part sections at and above the layer (3-axis access from the setup face);
* `left = R ∩ offset(offset(shadow, +r), −r)`.

The flat end mill of radius `r` reaches exactly the morphological opening of the free region (the plane less the
shadow). This opening is the union of all cutter disks that do not touch the part. Its complement is the closing of
the shadow. So the material left is the inside corners (radius `r` by construction), the slots narrower than `2r`
and the undercuts. The formula uses no pass, no toolpath and no 3D Boolean.

### Cutters with a profile

A ball end mill, a bull nose and a drill point have the radius `ρ(h)` at the height `h` above the tip.
The profile height `c` is `r` (ball), the corner radius (bull nose) or `r / tan(point angle / 2)` (drill).
With the tip at the height `t`, the cutter centre must stay out of

`obstacle(t) = ∪ over the layers above t of offset(part layer, ρ(layer top − t))`.

At the height `z` the cutter reaches the union over the tips `t` in `[z − c, z]` of `offset(free centres(t), ρ(z − t))`.
A tip lower than `z − c` gives no more reach: its obstacle is larger and its radius is the same. So

`left(z) = R ∩ ⋂ over t of offset(obstacle(t), −ρ(z − t))`.

The tips are 8 even steps of the profile height and every horizontal face (floor) in it. The profile height of each
layer is divided into 8 bands, each one evaluated at its middle. The error is about 1 % of the fillet volume
(test: ball D6 in a pocket corner, bull nose D6 r1).

Where the part below `z` adds nothing to the shadow of `z` near the region (a straight hole wall, also a modelled
thread), a profile cutter acts as a flat cutter with its tip at `z`.

### Drills

A drill makes holes (`DrillReach`). A hole is a small spot of free tool centres under the shadow of a layer: a
closed hole at most 10 % larger than the drill. The drill does not take out a pocket, an open groove or the open
stock by plunge milling. The layers are worked from the top down, so the drill finds each hole where its full disk
fits. For each hole axis the deepest tip `t*` is a 1D search: the point goes down while `ρ(z − t) ≤ d(z) − fit` at
the middle of every layer, where `d(z)` is the distance of the axis to the part section. The reach at the height `z`
is the disk of radius `ρ(z − t*)`. A modelled drill point of the same angle is reached exactly. The drill point
leaves its cone at a flat floor (for example a counterbore floor with a smaller hole below it).

A drill cutter can also carry the plunge points of its passes (`Cutter.plunges`, part frame, any point on each drill
axis). Each plunge is a hole of the drill radius down to the deepest tip that fits, with the point cone; the fit is
checked from the tip up to the top. An open groove or a slot made by repeated plunges is reached; the region left
is the cusps between the plunges. A spot face that is open at its side (the part around it is lower, so the free
centres are not a closed spot) is found only through its plunge point.

### Reach

A layer deeper below the setup face than `min(flute length, stick-out, reach)` is out of reach for that cutter.
The setup face is the part top in the setup frame. The plan faces it first (D2O "Face and square first datums"),
and the old replay measures the cutting depth from the incoming material top. Each reach depth adds a layer limit.
A tip can go below the stock bottom by the profile height (a drill point breaks through).

### Several cutters and several setups

In one setup, the cutters work in turn: flat cutters first, the largest first. Each next cutter sees only the
material that the earlier cutters leave. `left_by_cutter_mm3` is the left volume after each cutter, in this order.

Several setups: the material that no setup reaches is the left material of the reference setup (the main setup)
that every other setup also leaves. The other setups are cut on the plane of each reference layer:
a parallel setup gives its layer polygons; a side setup gives rectangles (its polygon intervals on the plane line
times its layer height). This combination needs setups along the part axes (the six D2O setup faces).

### Layers

* The layer limits are the heights of the horizontal faces of the stock and the part, and the cutter reach depths.
* A sloped or curved face divides its interval so that its section moves by at most 0.01 mm in one layer,
  but no layer is thinner than 0.1 mm (as the sloped layers of the layer replay). Above 400 layers the bound
  grows up to 0.05 mm (thinnest layer 0.5 mm). A triangle whose section moves less than the bound over its
  whole height does not divide its interval (the short triangles at the top of a horizontal cylinder).
* A setup other than the reference divides its sloped intervals only within its deepest cutter reach.
  Deeper, no cutter of that setup cuts, so its left material is all the material and it adds no constraint.
* The material of a layer is its middle section. The shadow takes the sections at the bottom, the middle and
  the top of each layer.

### Speed

* The faces mesh in parallel.
* The reference setup divides its sloped layers only where some setup cuts (`reach_ranges`).
* A cutter takes its free centres only within its radius of the region (`_unreached`), and the offset uses the
  smaller of two outlines: the free centres grown by the radius, or the local obstacle shrunk by the radius.
* Before an offset, a loop that the offset closes is dropped (a hole narrower than twice the growth, an outer loop
  narrower than twice the shrink). This is exact, and it keeps the many small holes of a large section out.
* A layer band that leaves the shadow unchanged adds no offset. Equal inputs reuse their results.
* A part mesh of 50,000 triangles or more gets one process for each setup.

### Tolerances

| Value | mm | Reason |
|---|---|---|
| Mesh chord (`layers.MESH_DEFLECTION_MM`) | 0.001 | The part and stock sections. |
| Cutter fit (`FIT_MM`) | 0.002 | The obstacle uses `r − 0.002`: the mesh chords and the offset chords lie inside the true surfaces, so a hole of the cutter size fits. |
| Disk reach | `r − 0.001` | The cutter disk reaches the mesh chord of the part polygon, not past the true surface. |
| Obstacle arc tolerance | 0.001 | Inside the fit. |
| Contact tolerance (`layers.CONTACT_MM`) | 0.001 | Left material thinner than 1 µm does not count (as the layer replay). |

The corners of the left material are therefore up to 3 µm smaller than the exact radius `r` gives
(test: 0.4 % of the corner volume of a D3 cutter).

### Result

`part_verdict(stock, part, setups, cutters)` returns:

* `setups[]`: for each setup the layer count, the material to remove, the left volume, the left volume after each
  cutter and the 10 largest left regions (height range, xy box in the setup frame, volume);
* `left_after_all_setups_mm3` and `can_make` (no material is left after all setups).

`cutters` is one list for all setups or a dict {setup name: list}. A cutter is a `ToolSpec`, a `ToolSpec.to_dict()`
row or a `Cutter`.

## Player data (schema `subcad.layer_player.v1`)

The material at each moment of the program is `part ∪ material left at the end ∪ the regions that later passes
remove`. These sets do not overlap. The player shows a pass by hiding its removed region. The browser needs no
Boolean, and the server makes no 3D state.

```json
{
  "schema": "subcad.layer_player.v1",
  "units": "mm",
  "coordinate_units_per_mm": 1000,
  "display_tolerance_mm": 0.002,
  "setups": [{"name": ">Z", "setup_to_part": [[1,0,0],[0,1,0],[0,0,1]], "origin_mm": [0,0,0]}],
  "part": {"display_mesh": "<the part display mesh link>"},
  "left": [{"setup": ">Z", "bands": [{"z": [z0, z1], "loops": [[x0,y0,x1,y1,...], ...]}]}],
  "passes": [{"setup": ">Z", "operation_id": "op30", "pass_index": 1, "label": "...",
              "removed": [{"z": [z0, z1], "loops": [[...]]}]}]
}
```

* A band is a height range `z` (setup frame, mm) and closed loops. The loops are flat integer lists in µm.
  Outer loops are counter-clockwise, holes clockwise. Consecutive layers with the same loops are one band.
* The browser extrudes each band along the setup z from `z[0]` to `z[1]` and places it with `setup_to_part`
  (columns: the setup x, y and z directions in the part frame) and `origin_mm`.
* `passes` are in program order. Pass `k` shows: the part, `left`, and the `removed` bands of the passes after `k`.

The layer replay of SubCAD gives each pass state `player_layers` = `{"schema", "frame", "removed"}`
(`LayerReplay.player_layers`). Set `milling_layers.PLAYER_BREP_STATES = False` to stop the B-Rep state of each pass
before the last one (`state["shape"]` is then `None`). The default stays `True`, so the D2O pin keeps its STL states.
For an operation without a layer replay (drilling, surface milling), `layer_player.removal_bands(before, after, setup)`
makes the bands from two solids with mesh sections only.

## D2O changes

1. Pin SubCAD to the merge commit of this change (it includes #12).
2. The machining review (`machining_preview.py`, `machining_review_builder.build_review`):
   call `subcad.milling_opening.part_verdict(stock, target, setups, cutters)` with the plan stock, the registered
   STEP target (`machining_target.target_snapshot`), the setup faces of the plan and the owned cutters of each setup.
   The verdict replaces the pass replay checks and the nominal comparison for milling: can it be made, what is left
   and where, which cutter leaves it.
3. Keep the operation chain for the passes: it gives the toolpaths, and the layer replay gives the 2D removed region
   of each pass. Set `milling_layers.PLAYER_BREP_STATES = False`: no B-Rep state for each pass. Stop the STL export
   of each pass state (`machining_passes.retained_operation_passes`) and save one player document
   (`layer_player.document`) with the review. The program mode (the milling program time and the NC program,
   #729, #826) does not change, so the price does not change.
4. A pass of an operation without a layer replay (drilling, surface milling) gets its bands from
   `layer_player.removal_bands(before, after, setup)`.
5. Webapp `MillingPlayer`: read the player document, extrude the bands (Three.js `ExtrudeGeometry` from `Shape`
   with holes, or one triangle strip per loop), and hide the removed bands of each pass in turn.
