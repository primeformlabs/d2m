# Native slot replay

SubCAD 0.2.9 replaces the ideal SlotOp cut with selected-cutter replay.
The operation applies the compensated obround paths to the actual incoming material.
It retains each roughing pass and finishing pass with its material and move range.
The operation preserves preceding features and translated stock coordinates.

Slots support all six cardinal setup faces and rotation in the selected face.
A matching cutter follows the slot center line with exact cylindrical sweeps.
A smaller cutter uses clipped raster and boundary paths.
Curved boundary paths use linear segments and remain approximate.
The operation compares the removed region with the requested slot region.
The report identifies remaining cavity material and overcut volume.

Through slots use the actual incoming thickness along the selected cutter axis.
Cutter reach, flute length, projection, type, and width must permit the cut.
Invalid dimensions, missing tools, and oversized cutters stop the operation.
A planning path before apply() has no retained material proof.

Stock.slot accepts stepdown, finish_allowance, and chord_error in millimetres.
The default finish allowance is zero so a matching cutter can follow the center line.
A matching cutter cannot leave a positive radial finish allowance.

The focused geometry checks cover selected tools, six faces, rotation, through depth,
translated incoming material, previous holes, retained states, and failure reasons.
Manufacturing readiness and complete target tolerance remain unverified.
Qualified entry, feeds, spindle settings, fixtures, and machine clearance remain required.
