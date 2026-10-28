# Milling replay on 2.5D layers

SubCAD 0.2.24 replays the flat end mill passes of these operations on 2D layers:
profile pocket, profile cutout, slot, facing, machine around profile and profile contour.
The checks use 2D polygon math only. They do not use 3D Booleans or point-in-solid tests.

## Method

A 3-axis setup has one fixed tool axis. In the setup frame, each pass cuts at one tip height.

1. The incoming solid gets one triangle mesh (chord limit 1 µm).
2. The layer limits are the pass heights and the mesh vertex heights in the cut range.
3. Each layer holds one 2D polygon set: the plane section of the mesh at the layer middle.
   A sloped or curved face divides its interval into layers of 0.1 mm or less.
4. The sweep of a pass is the 2D Minkowski sum of its tool-centre polyline and the cutter disk.
   pyclipper makes it with the integer scale 1 unit = 1 nm and an arc tolerance of 0.25 µm.
5. A pass removes its sweep from each layer between its tip and its flute top.
   The removed volume of the pass is the area of (layer ∩ sweep) times the layer height, summed.
6. The operation region check compares the layer material before and after the operation:
   * remaining material: (material after ∩ target region), less the parts thinner than 1 µm;
   * overcut: (removed material less the target region), less the parts thinner than 1 µm.
   The 1 µm opening removes the slivers between two polygon chords of one arc.

## Material ledger (SubCAD 0.2.26)

The layers of an operation do not come from the B-Rep result of the operation before.
They come from the material ledger (`material_ledger.py`):

    material = base − cut 1 − cut 2 − ...

* The base is the first solid of the chain (most of the time the stock block).
* A cut is a 2D region of one setup frame between two heights of that frame.
  Each cutting level of a layer operation is one cut: the union of its sweeps, from the tip to the flute top.
  Each drill plunge is one cut for the full diameter and one cut for each 0.1 mm band of its point
  (the radius at the band middle).
* The layer at the height h is the base mesh section less the cut sections at h (2D differences):
  * a cut with the same tool axis (for example >Z and <Z) is its region with the axes turned;
  * a cut with a crossing tool axis (for example >X in a >Z layer) is a set of rectangles:
    the chords of its region on the line of h times its height range.
* The cut heights are layer limits. A cut edge that slopes across the layer direction gets the
  0.1 mm layers, as a sloped face of the base does.

The ledger is attached to the result workplane of each operation. An operation that does not record
its cuts returns a workplane without a ledger. The next layer operation then starts a new ledger
with that B-Rep as its base.

So a damaged B-Rep state cannot change the layers, the removed volumes or the region checks of the
operations after it. TP-0127 (D2O): the 6 mm corner rest pass left a B-Rep that Booleans cannot cut
(the D22 hole wall is the same circle as the boss wall that the rest pass cut). The D22 hole now
gets its layers from the ledger and is ready.

## Material result

Profile pocket and outside profile planning clip the roughing paths once in the XY plane.
Each depth uses the same paths with its cutting height.
The rapid and retract moves keep the same safe height.
The finishing pass keeps its separate clipping and allowance.

The next operation and the display need a B-Rep. The operation result is the incoming solid less one
prism for each cutting level. A level prism holds only the part of its sweep region that no deeper level
covers. Most operations need one or two Booleans. The old replay needed one Boolean for each pass
and four Booleans for the region check.

The prism faces get exact arcs back: a run of polygon chords on one cutter circle becomes one
circular edge. If the arcs fail, plain polygon prisms (chords within 0.25 µm) replace them.
The B-Rep result volume must agree with the layer removal within the known errors of the two models
(SubCAD 0.2.33, D2O #1334, `LayerReplay.brep_tolerance`):

* layer midpoints and polygon chords of the cutter arcs: 0.5 % of the removed volume (at least 0.001 mm³);
* mesh chords: the layers cut a mesh whose chords are at most 1 µm (`MESH_DEFLECTION_MM`) from the true faces.
  The exact removal can differ only in that band along the outline of each removed layer region:
  1 µm × outline length × layer height;
* prism outline cleaning (level roughing, `BREP_CLEAN_MM` = 10 µm): the area between the raw and the cleaned
  outline of each level, inside the material outline, times the material height above the level.

The last pass report of level roughing has `material_brep_check`: the removed volume, each part of the tolerance,
the tolerance and the B-Rep difference.

Example (D2O #1334, PT-002334): two flats 8 mm across on a D10 turned section. The exact B-Rep removes 0.244 mm³ more
than the layers (0.021 mm³ from the mesh chords of the round stock, 0.223 mm³ from the 10 µm outline cleaning). The old
0.5 % limit (0.20 mm³) refused it and the result was the layer prisms, which the nominal comparison cannot use.
The tolerance is now 0.548 mm³, and the B-Rep is the result. A Boolean that misses one flat (20 mm³) is still refused.

If the Booleans fail or do not agree, the layers stay the result and the B-Rep comes from the ledger
(`MaterialLedger.solid`, no Boolean): one prism for each run of equal layers over the full height,
with the 2 µm outline of the player. The last pass report has `material_brep`: `chained_brep` or
`layer_prisms`. A later layer operation on a layer B-Rep makes its result from the ledger at once.
The layer B-Rep is a compound of stacked prisms: the nominal comparison (one solid) cannot use it.
`compare_nominal_solids` then stops with a clear error: the result is the layer model of a milling operation
because the B-Rep Boolean of the cut failed.
When the result uses ledger prisms, the replay makes them before it calculates cutter arcs for display Booleans.
Those arcs are unnecessary for the ledger solid.
If the layers cannot be made (an open mesh of a new ledger base), the operation uses the B-Rep replay.

## Operation player states

The state after each pass is the operation result plus the layer material that later passes
remove, as extruded layer polygons. The layers between two cutting levels show as one prism of
their middle layer. The outline tolerance of these display prisms is 2 µm.
These states are for the player only. The checks do not use them.

## Accuracy

The layer sampling budget can make slope layers thicker.
Required mesh and cut heights remain separate intervals, even when their count exceeds the sampling budget.
The slope loop uses that minimum interval count as its lower bound.
Thus, many required heights cannot cause an infinite sampling loop.

* A polygon chord is at most 0.25 µm inside its arc. The removed volume of a pass can differ from
  the exact sweep by at most 0.25 µm times the arc part of its boundary area.
* The remaining-material and overcut figures do not count material thinner than 1 µm.
  A thin crescent between a linearized path and a curved wall loses its thin ends.
* A sloped face uses the middle section of each 0.1 mm layer.

Program mode (D2O #729) is not changed. It keeps one cut of the target region for each operation.
