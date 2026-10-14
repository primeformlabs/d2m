# Native hole finishing

SubCAD 0.2.11 preserves the selected setup face for `Stock.bore()` and `Stock.ream()`.
Both operations use the actual incoming material.

## Circular milling

`Stock.bore()` uses a selected flat end mill and a compensated circular pocket path.
Each roughing pass and finishing pass retains its material shape and move range.
The operation records remaining target material and overcut volume.
The circular path uses linear segments when the cutter is smaller than the hole.
The result does not verify the requested size or roundness tolerance.

This operation implements circular milling.
A boring head and a boring bar require separate cutter models and qualification.
An unapplied operation has no synthetic replay path.

## Reaming

`Stock.ream()` requires a `reamer` with the requested cutting diameter.
The cutter requires a recorded lead angle and a positive lead tip diameter.
Its lead tip diameter must be smaller than its cutting diameter.
The complete lead must fit an existing pilot hole in the incoming material.
The native B-rep check rejects contact between the pilot corridor and the incoming material.

Blind depth refers to the full-diameter depth.
The lead extends below that depth.
Through operations use the incoming thickness along the selected cutter axis.
Flute length, reach, and projection must include the complete lead travel.

The native replay uses the rotational lead envelope.
The independent tri-dexel replay uses the same recorded lead dimensions.
Neither replay qualifies the flutes, runout, feed, finishing allowance, fixture, or requested hole tolerance.
Catalog reamers have unknown lead dimensions until a specification supplies them.
The engine cannot substitute a drill or a different reamer diameter.

## Evidence

`test_subcad_hole_finishing.py` passes 25 checks.
The checks cover all six setup faces, actual pilot material, retained passes, reach, and specific failures.
They also cover owned-tool selection, preceding holes, and independent reamer replay.
The affected drilling checks pass 19 checks.
Complete Milling coverage and manufacturing qualification remain open in issue #5.
