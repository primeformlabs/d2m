# Laser material replay

SubCAD 0.2.14 provides `replay_laser_material` for supplied compensated XY paths.
The caller must declare millimetres and the stock-local coordinate frame.
The caller supplies the effective kerf width. SubCAD does not change compensation or path order.

The engine subtracts a circular beam footprint through the actual material thickness.
Each path retains the pierce and segment material states.
The cut preserves preceding holes and the source material.
Loose pieces remain in the result until an explicit release.

`laser_selection_region` builds a reference for part selection.
`release_laser_part` selects an actual detached material solid. It never substitutes the reference geometry.
The release preserves the remaining sheet and loose pieces separately.
An open cut or ambiguous selection cannot produce a released part.

The report identifies air motion without a material removal claim.
Invalid coordinates, zero kerf, unsupported units, and missing native material produce explicit errors.

The model assumes a constant circular kerf. It does not verify heat, pierce quality, head clearance, or manufacturing tolerances.
The existing fluent profile operations remain separate from this path replay API.
Production callers must use the replay API with their checked paths and saved settings.

SubCAD 0.2.15 exports a disposable copy of the material to STL.
OCCT tessellation can change shared triangulations and invalidate a later live material state.
The eight-hole regression preserves valid material and detached slugs through every pierce and cut export.
Fourteen focused native checks pass. The model remains unqualified for physical manufacturing.
