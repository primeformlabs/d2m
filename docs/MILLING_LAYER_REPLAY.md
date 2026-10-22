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

## Material result

The next operation needs a B-Rep. The operation result is the incoming solid less one prism for
each cutting level. A level prism holds only the part of its sweep region that no deeper level
covers. Most operations need one or two Booleans. The old replay needed one Boolean for each pass
and four Booleans for the region check.

The prism faces get exact arcs back: a run of polygon chords on one cutter circle becomes one
circular edge. If the arcs fail, plain polygon prisms (chords within 0.25 µm) replace them.
The B-Rep result volume must agree with the layer removal within 0.5 %.
If it does not agree, or if the mesh is not closed, the operation uses the B-Rep replay.

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
