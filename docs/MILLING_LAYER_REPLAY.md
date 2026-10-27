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

The next operation and the display need a B-Rep. The operation result is the incoming solid less one
prism for each cutting level. A level prism holds only the part of its sweep region that no deeper level
covers. Most operations need one or two Booleans. The old replay needed one Boolean for each pass
and four Booleans for the region check.

The prism faces get exact arcs back: a run of polygon chords on one cutter circle becomes one
circular edge. If the arcs fail, plain polygon prisms (chords within 0.25 µm) replace them.
The B-Rep result volume must agree with the layer removal within 0.5 %.

If the Booleans fail or do not agree, the layers stay the result and the B-Rep comes from the ledger
(`MaterialLedger.solid`, no Boolean): one prism for each run of equal layers over the full height,
with the 2 µm outline of the player. The last pass report has `material_brep`: `chained_brep` or
`layer_prisms`. A later layer operation on a layer B-Rep makes its result from the ledger at once.
The layer B-Rep is a compound of stacked prisms: the nominal comparison (one solid) cannot use it.
When the result uses ledger prisms, the replay makes them before it calculates cutter arcs for display Booleans.
Those arcs are unnecessary for the ledger solid.
If the layers cannot be made (an open mesh of a new ledger base), the operation uses the B-Rep replay.

## Operation player states

The state after each pass is the operation result plus the layer material that later passes
remove, as extruded layer polygons. The layers between two cutting levels show as one prism of
their middle layer. The outline tolerance of these display prisms is 2 µm.
These states are for the player only. The checks do not use them.

## Accuracy

* A polygon chord is at most 0.25 µm inside its arc. The removed volume of a pass can differ from
  the exact sweep by at most 0.25 µm times the arc part of its boundary area.
* The remaining-material and overcut figures do not count material thinner than 1 µm.
  A thin crescent between a linearized path and a curved wall loses its thin ends.
* A sloped face uses the middle section of each 0.1 mm layer.

Program mode (D2O #729) is not changed. It keeps one cut of the target region for each operation.
