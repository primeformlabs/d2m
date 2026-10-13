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
python -m pip install --no-deps dist/d2o_subcad-0.1.0-py3-none-any.whl
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

The native Turning audit found unchanged material for facing, outside turning, boring, grooving, and threading.
These APIs currently record the operation through `PureToleranceOp`.
They do not provide usable native Turning material states.
The operation plan and its raster toolpath do not prove a completed Turning operation.
Turning geometry and lathe toolpath replay require further implementation.
