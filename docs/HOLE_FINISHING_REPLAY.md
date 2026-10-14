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

## Pilot preparation in 0.2.12

`Stock.ream_hole()` selects the required reamer before it selects the pilot drill.
The pilot drill must clear the recorded lead tip and leave material for the final diameter.
The selected drill requires sufficient reach for its complete point.
The method retains both cutter records and both material states.
An existing clear pilot hole needs no new drill.
Missing pilot tools cannot cause a full-size drill fallback.

Blind holes can supply an entry plane in the selected setup frame.
The method includes the incoming stock allowance above that plane in its cutting depth.
The method rejects entry planes outside the incoming material bounds.
The card must report a pilot point that extends below the finished reamer lead.
Finishing allowance and the required blind floor remain unqualified.

Fourteen preparation checks pass.
They cover all six setup faces, selected tools, existing pilots, missing tools, reach, and recorded blind entry planes.

## Earlier hole-finishing evidence

`test_subcad_hole_finishing.py` passes 25 checks.
The checks cover all six setup faces, actual pilot material, retained passes, reach, and specific failures.
They also cover owned-tool selection, preceding holes, and independent reamer replay.
The affected drilling checks pass 19 checks.
Complete Milling coverage and manufacturing qualification remain open in issue #5.
