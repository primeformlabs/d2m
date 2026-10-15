# Ball cutter replay

SubCAD 0.2.8 corrects ball-cutter removal for the default 6 mm dome cut.

The previous replay joined clipped spherical sweeps into one removal tool.
That union could produce invalid topology or lose earlier removal regions.
Re-cutting an already cleared spherical boundary could also create a false material sliver.
A valid-looking result could therefore restore removed material.

The ball cutter and spherical sweep now use single revolved meridians.
The sweep preserves the selected radius, endpoints, and exact capsule volume.
The cutter preserves its lower hemisphere and upper flute dimensions.

The replay applies each complete selected-cutter solid to the actual incoming material.
It checks actual contact before it runs a cut.
An empty contact leaves the material unchanged.
The Boolean operations preserve their input shapes.
Invalid topology or added material stops the pass with a specific reason.

Every surface row starts from the preceding row's material.
The replay cannot replace it with the original stock.
Each row retains its material, tool moves, removal report, and access face.

Linear spherical sweeps remain exact cutter geometry.
Variable-height moves with axial cutter contact still use bounded pose sampling.
Their reports preserve that approximation.
The reported material does not prove complete target tolerance, fixture clearance, or manufacturing readiness.

The focused checks cover capsule volumes and endpoints, cutter volume, empty contact, added-material rejection, and the default dome target.
They also cover previous holes, translated coordinates, axial contact, and retained material progression.
