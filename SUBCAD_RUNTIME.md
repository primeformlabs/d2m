# SubCAD runtime package

SubCAD is the geometry and operation simulation engine for D2O process cards.
The `d2o-subcad` package contains SubCAD, its simulation modules, and its manufacturing material rules.
It excludes training code, model files, generated data, and web applications.

Build the runtime wheel:

```text
python -m pip wheel --no-deps . --wheel-dir dist
```

Install the wheel in the CAD environment:

```text
python -m pip install --no-deps dist/d2o_subcad-0.2.24-py3-none-any.whl
```

Use `from subcad import Stock` from an installed package.
Source-checkout imports with `from src.subcad import Stock` remain available.
The package requires CadQuery, NumPy, trimesh, SciPy, rtree, and pyclipper.
The optional `video` dependencies support rendered video export.

Version 0.2.24 replays flat end mill passes on 2.5D material layers (`docs/MILLING_LAYER_REPLAY.md`).
This applies to profile pockets, profile cutouts, slots, facing, machine-around profiles and contours.
The removed volume of each pass and the operation region check use 2D polygon areas times layer heights.
These checks use no 3D Boolean and no point-in-solid test.
The operation result uses one Boolean for each cutting level that adds material removal.
The old replay used one Boolean for each pass and four Booleans for the region check.
The B-Rep replay remains for material that the layer model cannot represent.

Version 0.2.22 applies ruled loft Milling on all six cardinal setup faces.
Section `cx` and `cy` use the existing face-profile convention.
Section `z_mm` is the signed position along the outward cutter axis.
The operation transforms actual incoming material into the setup frame for cutter replay.
It transforms every retained material state and tool move into stable part coordinates.
The final operation result uses the exact last retained shape.
Nominal Boolean comparisons use a common local origin and reject invalid results.
This avoids a CadQuery 2.8 comparison failure on translated material with a preceding hole.
The comparison transform cannot change the actual cutter result or increase the allowed overcut.
The toolpath records its cutter axis, selected face, section frame, and safe-plane coordinate frame.
Independent replay uses the world-coordinate path and the same selected cutter.
Other cutter axes and arbitrary loft sections still require another machining plan.

Version 0.2.21 includes part curvature in external circular loft wall spacing.
The previous flat-surface spacing rule could exceed the selected preview accuracy on a small circular part.
The new calculation checks adjacent cylindrical cutting envelopes and increases the finishing-path count where necessary.
It retains actual spacing, its ideal radial material bound, and the selected preview accuracy.
Planar wall paths also retain their ideal spacing bound.
The card explains that these ideal geometry bounds do not verify manufacturing tolerances.
An insufficient path budget produces a specific reason. It cannot increase the selected preview accuracy.
Mixed section types retain their specific geometry reason.

Version 0.2.20 uses exact native cylinder sweeps for linear flat-cutter moves with changing Z.
The selected cutter's leading side face and leading circular cap generate the added sweep volume.
The engine joins these swept faces with the initial cutter. It does not use the target as a cutting solid.
The sweep supports ascending and descending travel, arbitrary horizontal direction, and translated coordinates.
Continuous cylinder membership and native volume checks verify the sweep.
Ball-cutter moves keep their existing geometry and explicit sampling limits.

Profile clearing retains repeated cutter paths and their material states.
An exactly repeated path reuses the material difference from its earlier occurrence.
Only material subtraction occurs between these passes. The earlier cutter sweep cannot remove more material on repetition.
Each report identifies the earlier pass. The complete physical motion remains in the timeline.
The independent replay keeps the retained verification interval for exact loft wall sweeps.
Generator spacing, wall scallops, fixtures, and manufacturing tolerances remain unverified.

Version 0.2.19 adds loft wall-finishing paths through the selected flat cutter.
`finish_walls=True` is the default. `finish_walls=False` retains the explicit clearing mode.
The wall paths follow the accessible ruled surfaces and preserve the material below the loft base.
The native replay uses finite cutter poses and retains each finishing result.
The comparison reports the remaining material before and after wall finishing.
The independent replay preserves the finer retained motion interval when its spatial grid is coarser.
Invalid intervals and excessive motion work produce specific reasons before replay.
Finite cutter poses, wall scallops, fixture qualification, and manufacturing tolerances remain unverified.

Version 0.2.18 adds native `loft_mill()` replay for concentric ruled circle or rectangle sections.
Each section supplies `type`, `z_mm`, `cx`, `cy`, and either `diameter` or `length` and `width`.
Section Z positions increase from the base to the top. The section dimensions cannot increase toward the top.
The top must match the incoming stock datum. The base leaves existing material below it.
The selected flat end mill clears each level through actual compensated cutter paths.
The engine retains every roughing and finishing pass and its matching tool moves.
The reference loft supplies the comparison target. It does not replace the material result.
The native comparison reports overcut and remaining terraced wall material.
Shifted sections, mixed section types, overhangs, missing dimensions, unsupported geometry fields, and unreachable cutters produce specific reasons.
Version 0.2.19 adds wall finishing to this clearing model.
Other cutter axes, arbitrary loft sections, non-linear continuous cutter sweeps, and complete tolerance qualification remain incomplete.
The operation remains a preview and cannot authorize manufacturing release.

Version 0.2.16 adds `replay_bending_material()` for actual incoming sheet meshes.
It retains each ordered bend result and preserves closed surface topology and cut holes.
The model uses explicit bend angles, signed directions, inside radii, and support modes.
It reports STL seam joins within 0.00002 mm and refuses open material or excess coordinate displacement.
It rejects incomplete sequences, insufficient panel material, changed topology, and excessive volume change.
The model remains a kinematic preview. Material behaviour, finite flange boundaries, contact, springback, and manufacturing tolerances require qualification.
The existing split-and-rotate `press_brake_bend()` does not provide this retained radiused model.

Version 0.2.17 adds exact contour-phase Laser replay through `retain_segments=False`.
The default replay retains every segment as before.
Contour replay retains the pierce state and the complete cut state without changing the supplied path.
Closed convex paths use exact round outer offsets and inner line offsets.
Other paths use the same exact segment capsules in a grouped native cut.
Self-crossing paths cannot use the convex offset method.
Native comparisons show no missing or extra material relative to sequential segment cuts.
The measured 35-point contour takes 0.41 seconds, compared with 2.09 seconds before the change.
The model still does not qualify heat, pierce quality, head clearance, or manufacturing tolerances.

Pin the exact source commit in the production dependency.
Test the installed wheel outside the source checkout before updating that pin.
Do not use a development workspace path as the production dependency.

Rounded pockets use native circular corner profiles before material removal.
The geometry code preserves stock coordinates after preceding cuts.
Invalid profiles raise an error. They do not produce a square substitute pocket.

The current runtime has Milling and Turning operations, toolpaths, fixture checks, and operation timelines.
The presence of an operation does not prove its complete machining accuracy.
Each operation requires native geometry, toolpath replay, fixture clearance, and target comparison evidence.
D2O issues #255 and #260 track the customer requirements.
Combined routes must retain the actual material result, datum, and fixture change between operations.

Version 0.2 models actual material removal for facing, outside turning, boring, grooves, profiles, and generic V threads.
`Stock.round_bar()` supports signed X, Y, and Z spindle axes and a part datum.
The native cuts preserve the incoming material and preceding Milling features.
Each operation retains its actual material state.
The Turning timeline uses those native states and reports independent tool replay as incomplete.
Neutral Turning paths use spindle radius and axial coordinates. They do not use Milling raster paths.
The Milling postprocessor does not emit those Turning moves.

Thread geometry uses a generic V profile. Thread-standard verification remains incomplete.
Owned tool selection, roughing passes, compensation, fixture clearance, and independent lathe replay remain incomplete.
The operation plan does not claim an achieved tolerance without measurement or verification.

Milling pocket roughing uses the cutter radius and retains the corner material that its tool cannot remove.
The separate finish operation removes that material with its own tool.
A requested sharp inside corner receives a specific review reason.

Version 0.2.2 adds an isolated inventory scope for each preview:

```python
from subcad import Stock, ToolSpec, tool_inventory

with tool_inventory(owned_tools, revision=inventory_hash, strict=True):
    result = Stock.rectangular(30, 30, 20).pocket(
        width=12, length=15, depth=3, corner_radius=3)
```

The caller supplies `owned_tools` from its qualified physical assemblies.
The scope copies the tools and preserves the inventory revision in each selection result.
Concurrent previews do not change the shared catalogue.
Strict selection rejects missing tools and unlisted explicit cutters.
Flute length and projection must cover the required cutting depth.
A missing finishing cutter stops the operation.
Actual holder metadata remains in `ToolSpec.physical_assembly`.
Generated holder metadata has `owned=false` and `geometry_status=approximate`.
This selection scope does not verify feeds, fixtures, or manufacturing readiness.

Version 0.2.3 adds native fixed-axis surface cutter replay.
`surface_mill({"type": "plane", "height_mm": z})` cuts the incoming material to an absolute part height.
`dome_mill(radius, height)` uses a compensated ball end mill path.
Its base lies `height` below the incoming top surface.
The native result includes cutter scallops and the planned allowance.
It does not replace that result with an ideal dome.
The code preserves incoming holes and translated part coordinates.
Each roughing row and finishing row retains its material state.
Each exported pass has a separate mesh path and content hash.

Constant-height and supported spherical sweeps use exact cutter solids.
Other variable-height moves use pose sampling and report that approximation.
Boolean failures stop the operation. They do not return unchanged material as a machined result.
Surface plans keep achieved tolerance unknown and manufacturing verification incomplete.
The current native surface setup uses the positive Z cutter axis.
Other axes require explicit path transforms and further verification.

`replay_milling_independently()` uses the separate tri-dexel ray/cutter engine.
It replays cutting moves from the actual incoming material.
It preserves absolute part coordinates and ignores rapid moves for material removal.
Its target comparison reports sampled overcut and undercut.
Finite grid sampling does not prove the requested tolerance.
Fixture, holder clearance, feed, and spindle checks remain required.

Unresolved surface references remain unavailable.
Sweep, loft, hollow-bore, and wire operations currently have no native material-removal implementation.
They raise specific errors. Their public API names do not prove geometry coverage.
Mill-turn setup remains a non-cutting instruction.
Complete Milling coverage and independent Turning replay remain open in issues #5 and #3.

Version 0.2.4 corrects `profile_contour()`.
An outside contour removes the selected cutter channel outside the profile.
An inside contour removes the channel inside the profile and preserves cutter-limited corners.
The operation does not pocket the interior or discard exterior waste.
Use separate operations to remove that material.
Each depth pass and finishing pass retains its actual material state and matching tool moves.
The path uses cutter-radius compensation and the actual profile boundary.
It preserves previous holes, Turning material, and translated stock coordinates.
All six cardinal setup faces use stable part coordinates and the correct cutter axis.

Circle, rectangle, rounded rectangle, slot, and explicit polygon profiles have native contour paths.
Circular path sections use bounded linear segments and report that approximation.
Arc-chain source profiles retain their source sampling limit.
Invalid profiles, divided offsets, missing cutters, and insufficient cutter reach stop the operation.
Contour entry, waste removal, fixture clearance, feeds, spindle speed, and target tolerance require verification.
The independent tri-dexel engine now supports all six cardinal cutter axes in part coordinates.
It retains its finite sampling limits and cannot authorize manufacturing release.

Version 0.2.5 replaces ideal profile-pocket cuts with selected-cutter replay.
Rasters and boundary passes respect the actual profile and retained islands.
Each roughing and finishing pass retains its actual material and matching cutter moves.
The selected cutter leaves the inside corner material that it cannot reach.
A native operation-region comparison reports remaining target material and overcut volume.
That comparison does not verify the complete part tolerance or manufacturing release.

Tool selection checks the accessible cutter-center region, including triangular openings and island gaps.
Matching circular cutters can use a point or line center path for circles and slots.
Through depth uses the actual selected stock axis.
Whole-stock openings permit cutter travel across open stock edges.
Outside profile clearing removes waste through actual cutter passes instead of intersecting the ideal target.
Flat cutter results unify coplanar Boolean faces only when validity and volume remain unchanged.
Qualified entry, fixtures, feeds, spindle speed, complete target comparison, and other operation families remain incomplete.

Version 0.2.6 cuts facing operations through retained cutter passes on six cardinal setup faces.
`face_mill(depth, face_selector="<Z")` supports the bottom datum without moving the profile origin after previous cuts.
Flat end mills use their cutter geometry. Face mills use a declared nominal cutting envelope.
Face-mill insert geometry and runout remain unverified.
The recorded tool identity remains the actual selected assembly.
Each facing pass preserves its native material state, matching tool moves, and target-region comparison.
An explicit facing stepover cannot exceed the selected cutter's saved maximum stepover.
Qualified fixtures, feeds, spindle speed, insert geometry, and complete part tolerances remain required.
