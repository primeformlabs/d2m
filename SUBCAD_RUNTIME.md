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
python -m pip install --no-deps dist/d2o_subcad-0.2.6-py3-none-any.whl
```

Use `from subcad import Stock` from an installed package.
Source-checkout imports with `from src.subcad import Stock` remain available.
The package requires CadQuery, NumPy, trimesh, SciPy, and rtree.
The optional `video` dependencies support rendered video export.

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

Version 0.2.5 combines these contour paths with browser stock frames and the continuous Turning clock.
It preserves supplied spindle phase across consecutive operations with the same recorded insert and spindle conditions.
It records non-cutting transport without changing the original operation-level plan.
Tool, setup, speed, and direction changes still need recorded transition data.

Version 0.2.6 exports fixed stock frames for native contour and surface paths.
These paths retain their existing absolute part coordinates and their independent replay contract.
The export includes the actual incoming bounds and a cutting-tip origin.
The browser can replay every retained pass with its selected flat or ball cutter.

Version 0.2.7 binds native drilling and spot drilling to all six cardinal stock faces.
The path records a cutting-tip origin, the actual incoming bounds, and the selected drill point height.
Through drilling includes the stock thickness, the cone height, and a recorded 1 mm breakthrough distance.
This distance remains unverified against fixtures and machine travel.
The path retains each peck plunge and retract.
Missing point geometry, insufficient cutting length, and a mismatched drill diameter prevent motion export.

Blind drilling records the commanded cutting-tip depth.
The current blind-hole BRep still uses a flat cylindrical cavity.
Its `cad_drill_bottom: flat_intent` metadata records this difference from the physical conical drill point.
Spot drilling records `cad_drill_bottom: planning_only` because its retained CAD state has no material removal.
The browser keeps these limitations visible.
Neither case supplies independent manufacturing proof.
