# Native drilling replay

SubCAD 0.2.10 retains actual material for drilling, spot drilling, and each peck plunge.
The selected drill diameter, point angle, and tip diameter define its rotational cutting envelope.
The replay does not replace the cut with an ideal requested hole.
Unknown point angles cannot supply a drilling result.

Blind drill depth refers to the full-diameter cylindrical depth.
The recorded point extends below that depth.
The operation rejects a blind depth when its point passes through the incoming material.
A through cut uses actual incoming thickness along its selected cutter axis.
Reach, flute length, and projection must permit the complete tip travel.

Spot depth refers to the drill tip depth.
Spot drilling cuts a shallow point start and retains its own tool and material state.
Stock.drill records spotting as a separate operation before drilling.
Peck drilling retains each plunge with its matching moves and removed volume.
Each pass uses the preceding material and preserves preceding features.
Invalid contact, invalid remaining material, and added material stop the pass.

The operations support six cardinal setup faces and translated incoming stock.
The native path uses tool-tip positions in part millimetres.
Independent tri-dexel replay uses analytic drill-ray intersections and recorded point geometry.
It supports pointed and truncated points without substituting a default angle.
Finite grid and motion sampling do not prove complete target tolerance.

The drilling result remains unverified for manufacturing.
Flute, chisel edge, runout, entry, fixture, feed, spindle, and complete target checks remain required.
A nominal zero tip diameter carries an explicit reason when the physical tip diameter is unknown.
