"""B-Rep geometry backend for SubCAD subtractive CAD library.

Wraps CadQuery for all solid modeling operations. Every machining operation
in SubCAD ultimately calls functions in this module to modify the 3D solid.

Coordinate system: Z is up, top face is +Z. All cuts go downward (-Z).
"""

from __future__ import annotations

import math

from .profiles import normalize_profile_points, profile_bounds_xy
from .pocket_geometry import rectangular_pocket_cut as _native_rectangular_pocket_cut

# ---------------------------------------------------------------------------
# CadQuery import guard
# ---------------------------------------------------------------------------

_HAS_CADQUERY = False
try:
    import cadquery as cq
    from cadquery import exporters as cq_exporters
    _HAS_CADQUERY = True
except ImportError:
    pass

# ---------------------------------------------------------------------------
# trimesh import guard (for to_mesh)
# ---------------------------------------------------------------------------

_HAS_TRIMESH = False
try:
    import trimesh
    _HAS_TRIMESH = True
except ImportError:
    pass


# =============================================================================
#  Stock creation
# =============================================================================

def create_rectangular_stock(length: float, width: float, height: float) -> "cq.Workplane":
    """Create a rectangular prism stock block centered at origin.

    Length=X, Width=Y, Height=Z.
    Stock extends from (-length/2, -width/2, -height/2) to
    (length/2, width/2, height/2).
    """
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    return cq.Workplane("XY").box(length, width, height, centered=True)


def create_cylindrical_stock(diameter: float, height: float) -> "cq.Workplane":
    """Create a cylindrical stock. Diameter in XY, height along Z."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    return cq.Workplane("XY").cylinder(height, diameter / 2.0)


def press_brake_bend(
    shape: "cq.Workplane",
    *,
    axis: str = "X",
    position: float = 0.0,
    angle: float = 90.0,
    side: str = "positive",
    radius: float = 1.0,
    axis_z: float | None = None,
) -> "cq.Workplane":
    """Approximate a press-brake bend by splitting and rotating one sheet side.

    The operation is intended for laser-cut sheet blanks modeled as thin
    solids in the XY plane. It preserves any cut holes or slots on the bent
    side because the actual B-Rep half is transformed.
    """
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")

    axis_name = str(axis or "X").upper()
    if axis_name not in {"X", "Y"}:
        raise ValueError("press brake bend axis must be 'X' or 'Y'")

    if abs(float(angle)) <= 1e-9:
        return shape

    moving_side = str(side or "positive").lower()
    positive = moving_side in {"+", "positive", "pos", "right", "top", "upper"}
    if not positive and moving_side not in {"-", "negative", "neg", "left", "bottom", "lower"}:
        raise ValueError("press brake bend side must be positive/+ or negative/-")

    bbox = shape.val().BoundingBox()
    x_min, x_max = bbox.xmin, bbox.xmax
    y_min, y_max = bbox.ymin, bbox.ymax
    z_min, z_max = bbox.zmin, bbox.zmax
    span_x = max(x_max - x_min, 1.0)
    span_y = max(y_max - y_min, 1.0)
    span_z = max(z_max - z_min, 1.0)
    pad = max(span_x, span_y, span_z, abs(float(radius)) * 4.0, 10.0)
    z_center = (z_min + z_max) / 2.0
    z_height = span_z + 2.0 * pad
    split = float(position)

    def y_box(y0: float, y1: float) -> "cq.Workplane | None":
        if y1 <= y0 + 1e-9:
            return None
        return (
            cq.Workplane("XY")
            .box(span_x + 2.0 * pad, y1 - y0, z_height, centered=True)
            .translate(((x_min + x_max) / 2.0, (y0 + y1) / 2.0, z_center))
        )

    def x_box(x0: float, x1: float) -> "cq.Workplane | None":
        if x1 <= x0 + 1e-9:
            return None
        return (
            cq.Workplane("XY")
            .box(x1 - x0, span_y + 2.0 * pad, z_height, centered=True)
            .translate(((x0 + x1) / 2.0, (y_min + y_max) / 2.0, z_center))
        )

    if axis_name == "X":
        moving_box = y_box(split, y_max) if positive else y_box(y_min, split)
        fixed_box = y_box(y_min, split) if positive else y_box(split, y_max)
        start = (x_min - pad, split, axis_z if axis_z is not None else z_max)
        end = (x_max + pad, split, axis_z if axis_z is not None else z_max)
    else:
        moving_box = x_box(split, x_max) if positive else x_box(x_min, split)
        fixed_box = x_box(x_min, split) if positive else x_box(split, x_max)
        start = (split, y_min - pad, axis_z if axis_z is not None else z_max)
        end = (split, y_max + pad, axis_z if axis_z is not None else z_max)

    if moving_box is None:
        return shape

    moving = shape.intersect(moving_box)
    rotated = moving.rotate(start, end, float(angle))
    if fixed_box is None:
        return rotated
    fixed = shape.intersect(fixed_box)
    return fixed.union(rotated)


def create_tapered_cylinder(
    bottom_diameter: float,
    top_diameter: float,
    height: float,
    *,
    inner_bottom_diameter: float = 0.0,
    inner_top_diameter: float = 0.0,
    z_center: float = 0.0,
) -> "cq.Workplane":
    """Create a conical/frustum disk or ring centered on the Z axis.

    Diameters are measured at the lower and upper Z faces.  Optional inner
    diameters create an axisymmetric bore/frustum, which keeps tapered extrudes
    representable as pure SubCAD turning intent instead of imported geometry.
    """
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    if height <= 0:
        raise ValueError("height must be positive")
    if bottom_diameter <= 0 or top_diameter <= 0:
        raise ValueError("outer diameters must be positive")

    z_min = z_center - height / 2.0
    outer = _make_cylinder_or_cone(bottom_diameter / 2.0, top_diameter / 2.0, height, z_min)

    if inner_bottom_diameter > 0 or inner_top_diameter > 0:
        ib = max(inner_bottom_diameter, 0.0)
        it = max(inner_top_diameter, 0.0)
        if ib >= bottom_diameter or it >= top_diameter:
            raise ValueError("inner diameters must be smaller than outer diameters")
        inner = _make_cylinder_or_cone(ib / 2.0, it / 2.0, height + 0.02, z_min - 0.01)
        outer = outer.cut(inner)

    return outer


def _make_cylinder_or_cone(
    bottom_radius: float,
    top_radius: float,
    height: float,
    z_min: float,
) -> "cq.Workplane":
    if abs(bottom_radius - top_radius) <= 1e-9:
        return (
            cq.Workplane("XY")
            .circle(bottom_radius)
            .extrude(height)
            .translate((0.0, 0.0, z_min))
        )
    return cq.Workplane("XY").add(
        cq.Solid.makeCone(
            bottom_radius,
            top_radius,
            height,
            cq.Vector(0, 0, z_min),
            cq.Vector(0, 0, 1),
        )
    )


def create_axis_aligned_tube(
    outer_diameter: float,
    inner_diameter: float,
    length: float,
    *,
    axis: str = "X",
    cx: float = 0.0,
    cy: float = 0.0,
    cz: float = 0.0,
    start: float | None = None,
    end: float | None = None,
) -> "cq.Workplane":
    """Create a straight tube along a principal axis from semantic dimensions."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    outer_diameter = float(outer_diameter)
    inner_diameter = float(inner_diameter or 0.0)
    length = float(length)
    if outer_diameter <= 0.0 or length <= 0.0:
        raise ValueError("tube outer diameter and length must be positive")
    if inner_diameter < 0.0 or inner_diameter >= outer_diameter:
        raise ValueError("tube inner diameter must be non-negative and smaller than outer diameter")

    axis_name = str(axis or "X").upper()
    if start is not None and end is not None:
        lo = min(float(start), float(end))
        hi = max(float(start), float(end))
        length = hi - lo
        center_axis = (lo + hi) / 2.0
    else:
        center_axis = {"X": cx, "Y": cy, "Z": cz}.get(axis_name, cx)
    if length <= 0.0:
        raise ValueError("tube start/end must span a positive length")

    if axis_name == "X":
        wp = cq.Workplane("YZ")
        translation = (center_axis - length / 2.0, cy, cz)
    elif axis_name == "Y":
        wp = cq.Workplane("XZ")
        translation = (cx, center_axis - length / 2.0, cz)
    elif axis_name == "Z":
        wp = cq.Workplane("XY")
        translation = (cx, cy, center_axis - length / 2.0)
    else:
        raise ValueError(f"unsupported tube axis: {axis!r}")

    profile = wp.circle(outer_diameter / 2.0)
    if inner_diameter > 0.0:
        profile = profile.circle(inner_diameter / 2.0)
    return profile.extrude(length).translate(translation)


def create_axis_aligned_profile_tube(
    outer_profile,
    inner_profile,
    length: float,
    *,
    axis: str = "X",
    cx: float = 0.0,
    cy: float = 0.0,
    cz: float = 0.0,
    start: float | None = None,
    end: float | None = None,
) -> "cq.Workplane":
    """Create a straight tube/duct from arbitrary outer and inner profiles."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    length = float(length)
    axis_name = str(axis or "X").upper()
    if start is not None and end is not None:
        lo = min(float(start), float(end))
        hi = max(float(start), float(end))
        length = hi - lo
        center_axis = (lo + hi) / 2.0
    else:
        center_axis = {"X": cx, "Y": cy, "Z": cz}.get(axis_name, cx)
    if length <= 0.0:
        raise ValueError("profile tube length/start/end must span a positive length")

    if axis_name == "X":
        wp = cq.Workplane("YZ")
        translation = (center_axis - length / 2.0, cy, cz)
    elif axis_name == "Y":
        wp = cq.Workplane("XZ")
        translation = (cx, center_axis - length / 2.0, cz)
    elif axis_name == "Z":
        wp = cq.Workplane("XY")
        translation = (cx, cy, center_axis - length / 2.0)
    else:
        raise ValueError(f"unsupported profile tube axis: {axis!r}")

    outer = _draw_profile(wp, outer_profile).extrude(length)
    if isinstance(inner_profile, dict) and str(inner_profile.get("type", "")).lower() in {
        "shell",
        "offset_shell",
        "wall",
    }:
        wall = float(
            inner_profile.get("wall_thickness")
            or inner_profile.get("wall_thickness_mm")
            or inner_profile.get("thickness")
            or 0.0
        )
        if wall <= 0.0:
            raise ValueError("profile tube shell wall_thickness must be positive")
        return outer.shell(-wall).translate(translation)
    if inner_profile:
        inner = _draw_profile(wp, inner_profile).extrude(length + 0.02).translate(
            (translation[0] - 0.01 if axis_name == "X" else translation[0],
             translation[1] - 0.01 if axis_name == "Y" else translation[1],
             translation[2] - 0.01 if axis_name == "Z" else translation[2])
        )
        return outer.translate(translation).cut(inner)
    return outer.translate(translation)


# =============================================================================
#  Helpers
# =============================================================================

def _to_face_coords(shape: "cq.Workplane", face_selector: str, cx: float, cy: float) -> tuple:
    """Convert SubCAD stock-centered coordinates to CadQuery face offsets.

    CadQuery's `.faces(...).workplane()` places the local origin from the
    selected face's workplane.  After previous cuts, that origin can drift
    away from the original stock origin, so subtract it to keep SubCAD
    operation coordinates stable in the stock/global XY frame.
    """
    if face_selector in {">Z", "<Z"}:
        try:
            origin = shape.faces(face_selector).workplane().plane.origin
            return cx - origin.x, cy - origin.y
        except Exception:
            pass
    return cx, cy


def _selected_face_workplane(shape: "cq.Workplane", face_selector: str) -> "cq.Workplane":
    """Build a workplane on selected faces, falling back for multi-face selections."""
    faces = shape.faces(face_selector)
    try:
        return faces.workplane()
    except Exception:
        return faces.workplane(centerOption="CenterOfMass")


def _side_drill_fallback(
    shape: "cq.Workplane",
    cx: float,
    cy: float,
    diameter: float,
    depth: float,
    through: bool,
    face_selector: str,
) -> "cq.Workplane":
    bb = shape.val().BoundingBox()
    selector = str(face_selector)
    radius = float(diameter) / 2.0
    if selector in {">Y", "<Y"}:
        length = bb.ylen + 2.0 if through else float(depth)
        origin_y = bb.ymax + 1.0 if through or selector == ">Y" else bb.ymin + length
        cutter = cq.Workplane("XZ").center(cx, cy).circle(radius).extrude(length).translate((0.0, origin_y, 0.0))
        return shape.cut(cutter)
    if selector in {">X", "<X"}:
        length = bb.xlen + 2.0 if through else float(depth)
        origin_x = bb.xmin - 1.0 if through or selector == "<X" else bb.xmax - length
        cutter = cq.Workplane("YZ").center(cx, cy).circle(radius).extrude(length).translate((origin_x, 0.0, 0.0))
        return shape.cut(cutter)
    raise ValueError(f"side drill fallback does not support face selector {face_selector!r}")


def _side_pocket_fallback(
    shape: "cq.Workplane",
    cx: float,
    cy: float,
    depth: float,
    face_selector: str,
    *,
    diameter: float | None = None,
    width: float | None = None,
    length: float | None = None,
) -> "cq.Workplane":
    """Cut a blind side-face pocket without relying on selected-face workplanes."""
    bb = shape.val().BoundingBox()
    selector = str(face_selector)
    cut_depth = max(float(depth), 0.0)
    if cut_depth <= 0.0:
        return shape

    if selector in {">Y", "<Y"}:
        origin_y = bb.ymax - cut_depth if selector == ">Y" else bb.ymin + cut_depth
        wp = cq.Workplane("XZ").center(cx, cy)
        if diameter is not None:
            cutter = wp.circle(float(diameter) / 2.0).extrude(cut_depth)
        else:
            cutter = wp.rect(float(length), float(width)).extrude(cut_depth)
        return shape.cut(cutter.translate((0.0, origin_y, 0.0)))

    if selector in {">X", "<X"}:
        origin_x = bb.xmax - cut_depth if selector == ">X" else bb.xmin + cut_depth
        wp = cq.Workplane("YZ").center(cx, cy)
        if diameter is not None:
            cutter = wp.circle(float(diameter) / 2.0).extrude(cut_depth)
        else:
            cutter = wp.rect(float(length), float(width)).extrude(cut_depth)
        return shape.cut(cutter.translate((origin_x, 0.0, 0.0)))

    raise ValueError(f"side pocket fallback does not support face selector {face_selector!r}")


def _side_slot_fallback(
    shape: "cq.Workplane",
    cx: float,
    cy: float,
    length: float,
    width: float,
    depth: float,
    through: bool,
    face_selector: str,
    angle: float = 0.0,
) -> "cq.Workplane":
    """Cut a side-face slot without relying on selected-face workplanes."""
    bb = shape.val().BoundingBox()
    selector = str(face_selector)
    if selector in {">Y", "<Y"}:
        cut_depth = bb.ylen + 2.0 if through else max(float(depth), 0.0)
        if cut_depth <= 0.0:
            return shape
        origin_y = bb.ymax + 1.0 if through or selector == ">Y" else bb.ymin + cut_depth
        wp = cq.Workplane("XZ").center(cx, cy)
        if angle:
            wp = wp.transformed(rotate=cq.Vector(0, 0, angle))
        cutter = wp.slot2D(float(length), float(width), 0).extrude(cut_depth)
        return shape.cut(cutter.translate((0.0, origin_y, 0.0)))

    if selector in {">X", "<X"}:
        cut_depth = bb.xlen + 2.0 if through else max(float(depth), 0.0)
        if cut_depth <= 0.0:
            return shape
        origin_x = bb.xmin - 1.0 if through or selector == "<X" else bb.xmax - cut_depth
        wp = cq.Workplane("YZ").center(cx, cy)
        if angle:
            wp = wp.transformed(rotate=cq.Vector(0, 0, angle))
        cutter = wp.slot2D(float(length), float(width), 0).extrude(cut_depth)
        return shape.cut(cutter.translate((origin_x, 0.0, 0.0)))

    raise ValueError(f"side slot fallback does not support face selector {face_selector!r}")




# =============================================================================
#  Machining operations
# =============================================================================

def face_mill_cut(shape: "cq.Workplane", depth: float, *, face_selector: str = ">Z") -> "cq.Workplane":
    """Remove material from the top face down to `depth`."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")

    if depth <= 0.001:
        return shape  # nothing to remove

    if face_selector not in {">X", "<X", ">Y", "<Y", ">Z", "<Z"}:
        raise ValueError("Face milling requires a cardinal stock face.")
    if not math.isfinite(depth):
        raise ValueError("The facing depth must be finite.")
    bbox = shape.val().BoundingBox()
    dimensions = [bbox.xlen, bbox.ylen, bbox.zlen]
    centers = [(bbox.xmin + bbox.xmax) / 2,
               (bbox.ymin + bbox.ymax) / 2,
               (bbox.zmin + bbox.zmax) / 2]
    normal = "XYZ".index(face_selector[1])
    sign = 1 if face_selector[0] == ">" else -1
    if depth >= dimensions[normal]:
        raise ValueError("The facing depth must preserve remaining stock.")
    centers[normal] += sign * (dimensions[normal] - depth) / 2
    dimensions = [value + 2.0 if index != normal else depth
                  for index, value in enumerate(dimensions)]
    cutter = cq.Workplane("XY").box(*dimensions).translate(tuple(centers))
    return shape.cut(cutter)


def rectangular_pocket_cut(
    shape: "cq.Workplane",
    cx: float,
    cy: float,
    width: float,
    length: float,
    depth: float,
    corner_radius: float = 0.0,
    *,
    face_selector: str = ">Z",
    base_height: float | None = None,
) -> "cq.Workplane":
    """Cut a rectangular pocket at (cx, cy) on the top face."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")

    return _native_rectangular_pocket_cut(shape, width=width, length=length, depth=depth,
        corner_radius=corner_radius, cx=cx, cy=cy, face_selector=face_selector, base_height=base_height)


def sloped_rectangular_cut(
    shape: "cq.Workplane",
    cx: float,
    cy: float,
    width: float,
    length: float,
    start_depth: float,
    end_depth: float,
    *,
    slope_axis: str = "X",
    face_selector: str = ">Z",
) -> "cq.Workplane":
    """Cut an axis-aligned rectangular pocket with a planar sloped floor.

    Depths are measured down from the current top face. ``slope_axis`` selects
    whether the floor changes along X/length or Y/width.
    """
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")

    width = float(width)
    length = float(length)
    start_depth = float(start_depth)
    end_depth = float(end_depth)
    if width <= 0.0 or length <= 0.0:
        raise ValueError("sloped cut width and length must be positive")
    if start_depth < 0.0 or end_depth < 0.0:
        raise ValueError("sloped cut depths must be non-negative")
    if max(start_depth, end_depth) <= 0.0:
        return shape
    if face_selector != ">Z":
        raise ValueError("sloped rectangular cuts currently support face_selector='>Z' only")

    bbox = shape.val().BoundingBox()
    z_top = bbox.zmax
    z_pad = min(max(max(start_depth, end_depth), 1.0), 5.0)
    axis = str(slope_axis or "X").upper()

    if axis in {"X", "LENGTH"}:
        x0 = cx - length / 2.0
        x1 = cx + length / 2.0
        profile = [
            (x0, z_top + z_pad),
            (x1, z_top + z_pad),
            (x1, z_top - end_depth),
            (x0, z_top - start_depth),
        ]
        cutter = (
            cq.Workplane("XZ")
            .polyline(profile)
            .close()
            .extrude(width / 2.0, both=True)
            .translate((0.0, cy, 0.0))
        )
    elif axis in {"Y", "WIDTH"}:
        y0 = cy - width / 2.0
        y1 = cy + width / 2.0
        profile = [
            (y0, z_top + z_pad),
            (y1, z_top + z_pad),
            (y1, z_top - end_depth),
            (y0, z_top - start_depth),
        ]
        cutter = (
            cq.Workplane("YZ")
            .polyline(profile)
            .close()
            .extrude(length / 2.0, both=True)
            .translate((cx, 0.0, 0.0))
        )
    else:
        raise ValueError("slope_axis must be 'X'/'length' or 'Y'/'width'")

    return shape.cut(cutter)


def circular_pocket_cut(
    shape: "cq.Workplane",
    cx: float,
    cy: float,
    diameter: float,
    depth: float,
    *,
    face_selector: str = ">Z",
    base_height: float | None = None,
) -> "cq.Workplane":
    """Cut a circular pocket at (cx, cy) on the top face."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")

    if base_height is not None:
        bb = shape.val().BoundingBox()
        z_min = bb.zmin + float(base_height)
        cut_depth = min(float(depth), bb.zmax - z_min)
        if cut_depth <= 0.0:
            return shape
        cutter = (
            cq.Workplane("XY")
            .center(cx, cy)
            .circle(diameter / 2.0)
            .extrude(cut_depth)
            .translate((0.0, 0.0, z_min))
        )
        return shape.cut(cutter)

    cx_rel, cy_rel = _to_face_coords(shape, face_selector, cx, cy)

    try:
        return (
            _selected_face_workplane(shape, face_selector)
            .center(cx_rel, cy_rel)
            .circle(diameter / 2.0)
            .cutBlind(-depth)
        )
    except Exception:
        if face_selector in {">Y", "<Y", ">X", "<X"}:
            return _side_pocket_fallback(
                shape, cx, cy, depth, face_selector, diameter=diameter
            )
        raise


def drill_hole(
    shape: "cq.Workplane",
    cx: float,
    cy: float,
    diameter: float,
    depth: float,
    through: bool = False,
    *,
    face_selector: str = ">Z",
) -> "cq.Workplane":
    """Drill a hole. If through=True, goes through entire part.
    Otherwise blind at `depth`."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")

    from .drill_geometry import drill_cut
    return drill_cut(shape, cx, cy, diameter, depth, through, face_selector=face_selector)


def countersink_cut(
    shape: "cq.Workplane",
    cx: float,
    cy: float,
    hole_diameter: float,
    sink_diameter: float,
    depth: float,
    sink_angle: float = 82.0,
    *,
    face_selector: str = ">Z",
) -> "cq.Workplane":
    """Cut a countersink: drill through-hole + conical recess.

    The through-hole is drilled first, then a shallow circular pocket
    creates the conical countersink recess.  The *sink_angle* parameter
    is recorded in the operation metadata but does not affect the
    simple CSG geometry.
    """
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    # Drill the through hole first
    result = drill_hole(shape, cx, cy, hole_diameter, depth=0, through=True,
                        face_selector=face_selector)
    # Then a shallow circular pocket for the countersink recess
    result = circular_pocket_cut(result, cx, cy, sink_diameter, depth,
                                 face_selector=face_selector)
    return result


def slot_cut(
    shape: "cq.Workplane",
    cx: float,
    cy: float,
    length: float,
    width: float,
    depth: float,
    angle: float = 0.0,
    through: bool = False,
    *,
    face_selector: str = ">Z",
) -> "cq.Workplane":
    """Cut a slot (obround) at (cx, cy) on the top face at given angle."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")

    cx_rel, cy_rel = _to_face_coords(shape, face_selector, cx, cy)
    try:
        wp = _selected_face_workplane(shape, face_selector).center(cx_rel, cy_rel)
    except Exception:
        if face_selector in {">Y", "<Y", ">X", "<X"}:
            return _side_slot_fallback(
                shape, cx, cy, length, width, depth, through, face_selector, angle
            )
        raise

    # Rotate the workplane if angle is non-zero
    if angle != 0.0:
        wp = wp.transformed(rotate=cq.Vector(0, 0, angle))

    slot_profile = wp.slot2D(length, width, 0)

    if through:
        return slot_profile.cutThruAll()
    else:
        return slot_profile.cutBlind(-depth)


def contour_cut_outer(
    shape: "cq.Workplane",
    depth: float,
    stepdown: float = 0.0,
    *,
    face_selector: str = ">Z",
) -> "cq.Workplane":
    """Trim the outer profile of the part by `depth` from the top face.

    This is a simplified contour -- in practice it means cutting the outer
    edges down. Uses a box-frame cutting tool that creates a step around the
    outer perimeter of the part.
    """
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")

    # TODO: non-Z contour needs reorientation logic.
    if face_selector != ">Z":
        pass  # face_selector accepted but contour logic is Z-axis based

    bb = shape.val().BoundingBox()
    part_length = bb.xmax - bb.xmin
    part_width = bb.ymax - bb.ymin
    margin = 10.0  # oversize margin to ensure cut covers part edges

    # Width of the contour step (inward offset from outer edges)
    contour_offset = 5.0
    inner_length = max(part_length - 2.0 * contour_offset, 1.0)
    inner_width = max(part_width - 2.0 * contour_offset, 1.0)

    def _make_frame_tool(frame_depth: float) -> "cq.Workplane":
        """Create a rectangular frame cutting tool of given depth."""
        outer = cq.Workplane("XY").box(
            part_length + 2.0 * margin,
            part_width + 2.0 * margin,
            frame_depth + 0.5,
            centered=(True, True, False),
        )
        inner = cq.Workplane("XY").box(
            inner_length,
            inner_width,
            frame_depth + 1.0,
            centered=(True, True, False),
        )
        return outer.cut(inner)

    if stepdown <= 0.0:
        # Single pass
        tool = _make_frame_tool(depth)
        tool = tool.translate(cq.Vector(0, 0, bb.zmax - depth))
        return shape.cut(tool)

    # Multiple passes with stepdown
    result = shape
    remaining = depth
    current_z = bb.zmax

    while remaining > 0.001:
        pass_depth = min(stepdown, remaining)
        tool = _make_frame_tool(pass_depth)
        tool = tool.translate(cq.Vector(0, 0, current_z - pass_depth))
        result = result.cut(tool)
        current_z -= pass_depth
        remaining -= pass_depth

    return result


def _profile_points(profile) -> list[tuple[float, float]]:
    return normalize_profile_points(profile)


def _profile_bounds(profile) -> tuple[float, float, float, float]:
    return profile_bounds_xy(profile)


def _draw_profile(wp: "cq.Workplane", profile) -> "cq.Workplane":
    if isinstance(profile, dict):
        profile_type = str(profile.get("type", profile.get("shape", ""))).lower()
        cx = float(profile.get("cx", profile.get("x", 0.0)))
        cy = float(profile.get("cy", profile.get("y", 0.0)))
        if profile.get("center") is not None:
            cx = float(profile["center"][0])
            cy = float(profile["center"][1])
        if profile_type in {"polygon", "regular_polygon"} or "n_sides" in profile or "sides" in profile:
            points = normalize_profile_points(profile)
            if len(points) >= 3:
                return wp.polyline(points).close()
        if (
            profile_type not in {"slot", "obround", "obround_slot", "slot2d"}
            and (profile_type in {"circle", "circular"} or "diameter" in profile or "radius" in profile)
            and not profile.get("segments")
        ):
            radius = float(profile.get("radius", float(profile.get("diameter", 10.0)) / 2.0))
            return wp.center(cx, cy).circle(radius).center(-cx, -cy)
        if profile_type not in {"slot", "obround", "obround_slot", "slot2d"} and (
            profile_type in {"rectangle", "rect", "rib", "pad"} or "width" in profile or "length" in profile
        ):
            width = float(profile.get("width", profile.get("length", 10.0)))
            length = float(profile.get("length", profile.get("width", 10.0)))
            if "angle" in profile or "rotation_deg" in profile:
                points = normalize_profile_points(profile)
                if len(points) >= 3:
                    return wp.polyline(points).close()
            return wp.center(cx, cy).rect(length, width).center(-cx, -cy)
    points = _profile_points(profile)
    if len(points) >= 3:
        return wp.polyline(points).close()
    return wp.rect(10.0, 10.0)


def profile_pocket_cut(
    shape: "cq.Workplane",
    profile,
    depth: float,
    *,
    face_selector: str = ">Z",
    through: bool = False,
    islands=None,
) -> "cq.Workplane":
    """Cut a closed arbitrary profile on the selected face."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    island_list = list(islands or [])
    if island_list:
        if not _is_rectangular_profile(profile) or len(island_list) > 1:
            return _cut_profile_region_around_islands(
                shape,
                outer_profile=profile,
                island_profiles=island_list,
                depth=depth,
                face_selector=face_selector,
                through=through,
            )
        # Common v1 case for Zero-to-CAD: pocket a rectangular stock/region
        # while retaining one or more rectangular islands (rib/pad/bosses).
        result = shape
        for island in island_list:
            result = _cut_rectangular_region_around_island(
                result,
                outer_profile=profile,
                island_profile=island,
                depth=depth,
                face_selector=face_selector,
                through=through,
            )
        return result
    wp = _draw_profile(shape.faces(face_selector).workplane(), profile)
    return wp.cutThruAll() if through else wp.cutBlind(-depth)


def profile_cutout_cut(
    shape: "cq.Workplane",
    profile,
    depth: float,
    *,
    face_selector: str = ">Z",
    through: bool = False,
) -> "cq.Workplane":
    """Cut outside a profile, retaining the requested part outline.

    ``profile_pocket_cut`` removes the inside of the profile.  A cutout for
    dataset translation usually means the opposite: start with stock and keep
    the closed 2D profile as the resulting blank.  Through cutouts are modeled
    as an intersection with a profile prism, which is much more robust than a
    nested-wire frame cut for arbitrary polygon profiles.
    """
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    bbox = shape.val().BoundingBox()
    current_height = bbox.zmax - bbox.zmin
    if through or depth >= current_height - 1e-6:
        if face_selector not in {">Z", "<Z"}:
            return shape.intersect(_profile_prism_through_face(shape, profile, face_selector))
        pad = 1.0
        prism = _draw_profile(
            cq.Workplane("XY").workplane(offset=bbox.zmin - pad),
            profile,
        ).extrude((bbox.zmax - bbox.zmin) + 2.0 * pad)
        return shape.intersect(prism)

    outer_profile = _stock_envelope_profile(shape, margin=0.0)
    if _is_rectangular_profile(profile) and not _is_rotated_profile(profile):
        return _cut_rectangular_region_around_island(
            shape,
            outer_profile=outer_profile,
            island_profile=profile,
            depth=depth,
            face_selector=face_selector,
            through=False,
        )
    wp = _draw_profile(shape.faces(face_selector).workplane(), outer_profile)
    wp = _draw_profile(wp, profile)
    return wp.cutBlind(-depth)


def _profile_prism_through_face(shape: "cq.Workplane", profile, face_selector: str) -> "cq.Workplane":
    """Create a through prism from a side-face profile.

    `profile_cutout(..., through=True)` means retain the supplied 2D profile.
    For side faces, the profile coordinates are interpreted on the selected
    face plane: X/Z for +/-Y faces and Y/Z for +/-X faces.
    """
    bbox = shape.val().BoundingBox()
    pad = 1.0
    face = str(face_selector or ">Z").upper()
    if face in {">Y", "<Y"}:
        length = (bbox.ymax - bbox.ymin) + 2.0 * pad
        y_center = (bbox.ymin + bbox.ymax) / 2.0
        return _draw_profile(cq.Workplane("XZ"), profile).extrude(length, both=True).translate(
            (0.0, y_center, 0.0)
        )
    if face in {">X", "<X"}:
        length = (bbox.xmax - bbox.xmin) + 2.0 * pad
        x_center = (bbox.xmin + bbox.xmax) / 2.0
        return _draw_profile(cq.Workplane("YZ"), profile).extrude(length, both=True).translate(
            (x_center, 0.0, 0.0)
        )
    pad_prism = _draw_profile(
        cq.Workplane("XY").workplane(offset=bbox.zmin - pad),
        profile,
    ).extrude((bbox.zmax - bbox.zmin) + 2.0 * pad)
    return pad_prism


def profile_contour_cut(
    shape: "cq.Workplane",
    profile,
    depth: float,
    *,
    side: str = "outside",
    face_selector: str = ">Z",
) -> "cq.Workplane":
    """Contour an arbitrary profile; current B-Rep cut uses profile pocket CSG."""
    return profile_pocket_cut(shape, profile, depth, face_selector=face_selector, through=False)


def thin_wall_pocket_cut(
    shape: "cq.Workplane",
    profile,
    wall_thickness: float,
    depth: float,
    *,
    face_selector: str = ">Z",
) -> "cq.Workplane":
    """Cut the accessible inner cavity for a lightweight thin-wall profile.

    This is a conservative v1 geometry implementation for translator coverage:
    it keeps the outer profile as stock and removes the inward-offset cavity.
    Rectangular and circular profiles are handled semantically; other profiles
    fall back to their bounding box until true polygon offsetting is added.
    """
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    wall = max(float(wall_thickness), 0.0)
    if isinstance(profile, dict):
        profile_type = str(profile.get("type", profile.get("shape", ""))).lower()
        cx = float(profile.get("cx", profile.get("x", 0.0)))
        cy = float(profile.get("cy", profile.get("y", 0.0)))
        if profile.get("center") is not None:
            cx = float(profile["center"][0])
            cy = float(profile["center"][1])
        if profile_type in {"circle", "circular"} or "diameter" in profile or "radius" in profile:
            radius = float(profile.get("radius", float(profile.get("diameter", 10.0)) / 2.0))
            inner_radius = max(radius - wall, 0.0)
            if inner_radius <= 0.0:
                return shape
            return (
                shape.faces(face_selector)
                .workplane()
                .center(cx, cy)
                .circle(inner_radius)
                .cutBlind(-depth)
            )
        if profile_type in {"rectangle", "rect", "box"} or "width" in profile or "length" in profile:
            width = float(profile.get("width", profile.get("length", 10.0)))
            length = float(profile.get("length", profile.get("width", 10.0)))
            inner_width = max(width - 2.0 * wall, 0.0)
            inner_length = max(length - 2.0 * wall, 0.0)
            if inner_width <= 0.0 or inner_length <= 0.0:
                return shape
            return (
                shape.faces(face_selector)
                .workplane()
                .center(cx, cy)
                .rect(inner_length, inner_width)
                .cutBlind(-depth)
            )

    xmin, ymin, xmax, ymax = _profile_bounds(profile)
    inner_length = max((xmax - xmin) - 2.0 * wall, 0.0)
    inner_width = max((ymax - ymin) - 2.0 * wall, 0.0)
    if inner_width <= 0.0 or inner_length <= 0.0:
        return shape
    cx = (xmin + xmax) / 2.0
    cy = (ymin + ymax) / 2.0
    return (
        shape.faces(face_selector)
        .workplane()
        .center(cx, cy)
        .rect(inner_length, inner_width)
        .cutBlind(-depth)
    )


def thin_wall_shell_cut(
    shape: "cq.Workplane",
    wall_thickness: float,
) -> "cq.Workplane":
    """Offset-shell the current shape to represent a thin-wall shell operation."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    wall = abs(float(wall_thickness))
    if wall <= 0.0:
        return shape
    return shape.shell(-wall)


def machine_around_profile_cut(
    shape: "cq.Workplane",
    profile,
    height: float,
    *,
    face_selector: str = ">Z",
    margin: float = 5.0,
    stock_envelope=None,
    base_height: float | None = None,
) -> "cq.Workplane":
    """Remove a frame around a retained profile island."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    if base_height is not None:
        outer_profile = _stock_envelope_profile(shape, stock_envelope, margin=margin)
        return _cut_profile_region_around_islands_z_band(
            shape,
            outer_profile=outer_profile,
            island_profiles=[profile],
            base_height=base_height,
            height=height,
        )
    if _is_rectangular_profile(profile) and not _is_rotated_profile(profile):
        outer_profile = _stock_envelope_profile(shape, stock_envelope, margin=margin)
        return _cut_profile_region_around_island(
            shape,
            outer_profile=outer_profile,
            island_profile=profile,
            depth=height,
            face_selector=face_selector,
            through=False,
        )
    if stock_envelope is not None or _is_circular_profile(profile) or _is_rotated_profile(profile):
        outer_profile = _stock_envelope_profile(shape, stock_envelope, margin=margin)
        return _cut_profile_region_around_island(
            shape,
            outer_profile=outer_profile,
            island_profile=profile,
            depth=height,
            face_selector=face_selector,
            through=False,
        )
    xmin, ymin, xmax, ymax = _profile_bounds(profile)
    outer_l = max(xmax - xmin + 2 * margin, 1.0)
    outer_w = max(ymax - ymin + 2 * margin, 1.0)
    cx = (xmin + xmax) / 2.0
    cy = (ymin + ymax) / 2.0
    wp = shape.faces(face_selector).workplane().center(cx, cy).rect(outer_l, outer_w).center(-cx, -cy)
    wp = _draw_profile(wp, profile)
    return wp.cutBlind(-height)


def machine_around_profiles_cut(
    shape: "cq.Workplane",
    profiles,
    height: float,
    *,
    face_selector: str = ">Z",
    margin: float = 5.0,
    stock_envelope=None,
    base_height: float | None = None,
) -> "cq.Workplane":
    """Remove material around multiple retained profile islands in one cut.

    Sequential single-island cuts can erase previously retained ribs/bosses.
    Cutting all islands in one sketch preserves patterned retained material.
    """
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    islands = [profile for profile in list(profiles or []) if profile]
    if not islands:
        return shape
    if base_height is not None:
        outer_profile = _stock_envelope_profile(shape, stock_envelope, margin=margin)
        return _cut_profile_region_around_islands_z_band(
            shape,
            outer_profile=outer_profile,
            island_profiles=islands,
            base_height=base_height,
            height=height,
        )
    if len(islands) == 1:
        return machine_around_profile_cut(
            shape,
            islands[0],
            height,
            face_selector=face_selector,
            margin=margin,
            stock_envelope=stock_envelope,
        )
    outer_profile = _stock_envelope_profile(shape, stock_envelope, margin=margin)
    return _cut_profile_region_around_islands(
        shape,
        outer_profile=outer_profile,
        island_profiles=islands,
        depth=height,
        face_selector=face_selector,
        through=False,
    )


def _is_rectangular_profile(profile) -> bool:
    if not isinstance(profile, dict):
        return False
    profile_type = str(profile.get("type", profile.get("shape", ""))).lower()
    if profile_type:
        return profile_type in {"rectangle", "rect", "box", "rib", "pad"}
    return "width" in profile or "length" in profile


def _is_rotated_profile(profile) -> bool:
    if not isinstance(profile, dict):
        return False
    return abs(float(profile.get("angle", profile.get("rotation_deg", 0.0)))) > 1e-12


def _is_circular_profile(profile) -> bool:
    if not isinstance(profile, dict):
        return False
    profile_type = str(profile.get("type", profile.get("shape", ""))).lower()
    return (
        profile_type in {"circle", "circular", "cylinder", "boss"}
        or "diameter" in profile
        or "radius" in profile
    )


def _stock_envelope_profile(shape: "cq.Workplane", stock_envelope=None, *, margin: float = 0.0) -> dict:
    """Return a rectangular profile describing the current/declared stock XY."""
    if isinstance(stock_envelope, dict) and (
        "length" in stock_envelope or "width" in stock_envelope
        or "length_mm" in stock_envelope or "width_mm" in stock_envelope
    ):
        length = float(stock_envelope.get("length", stock_envelope.get("length_mm", 0.0)))
        width = float(stock_envelope.get("width", stock_envelope.get("width_mm", 0.0)))
        cx = float(stock_envelope.get("cx", stock_envelope.get("x", 0.0)))
        cy = float(stock_envelope.get("cy", stock_envelope.get("y", 0.0)))
        if length > 0.0 and width > 0.0:
            return {"type": "rect", "length": length, "width": width, "cx": cx, "cy": cy}

    bb = shape.val().BoundingBox()
    length = max((bb.xmax - bb.xmin) + 2.0 * margin, 1.0)
    width = max((bb.ymax - bb.ymin) + 2.0 * margin, 1.0)
    return {
        "type": "rect",
        "length": length,
        "width": width,
        "cx": (bb.xmin + bb.xmax) / 2.0,
        "cy": (bb.ymin + bb.ymax) / 2.0,
    }


def _cut_rectangular_region_around_island(
    shape: "cq.Workplane",
    *,
    outer_profile,
    island_profile,
    depth: float,
    face_selector: str = ">Z",
    through: bool = False,
) -> "cq.Workplane":
    """Cut rectangular regions inside an outer rectangle but outside an island.

    This avoids fragile nested-wire sketches when an island touches the outer
    boundary, such as a rib that spans the full Y width of the stock.
    """
    oxmin, oymin, oxmax, oymax = _profile_bounds(outer_profile)
    ixmin, iymin, ixmax, iymax = _profile_bounds(island_profile)
    ixmin = max(ixmin, oxmin)
    ixmax = min(ixmax, oxmax)
    iymin = max(iymin, oymin)
    iymax = min(iymax, oymax)

    if ixmin >= ixmax or iymin >= iymax:
        return shape

    regions: list[tuple[float, float, float, float]] = []

    def add_region(x0: float, y0: float, x1: float, y1: float) -> None:
        if x1 - x0 > 1e-6 and y1 - y0 > 1e-6:
            regions.append(((x0 + x1) / 2.0, (y0 + y1) / 2.0, x1 - x0, y1 - y0))

    add_region(oxmin, oymin, ixmin, oymax)  # left of island
    add_region(ixmax, oymin, oxmax, oymax)  # right of island
    add_region(ixmin, oymin, ixmax, iymin)  # below island
    add_region(ixmin, iymax, ixmax, oymax)  # above island

    if not regions:
        return shape

    wp = shape.faces(face_selector).workplane()
    for cx, cy, length, width in regions:
        wp = wp.center(cx, cy).rect(length, width).center(-cx, -cy)
    return wp.cutThruAll() if through else wp.cutBlind(-depth)


def _cut_profile_region_around_island(
    shape: "cq.Workplane",
    *,
    outer_profile,
    island_profile,
    depth: float,
    face_selector: str = ">Z",
    through: bool = False,
) -> "cq.Workplane":
    """Cut an outer profile while retaining an arbitrary inner island."""
    return _cut_profile_region_around_islands(
        shape,
        outer_profile=outer_profile,
        island_profiles=[island_profile],
        depth=depth,
        face_selector=face_selector,
        through=through,
    )


def _cut_profile_region_around_islands(
    shape: "cq.Workplane",
    *,
    outer_profile,
    island_profiles,
    depth: float,
    face_selector: str = ">Z",
    through: bool = False,
) -> "cq.Workplane":
    """Cut an outer profile while retaining multiple inner islands."""
    if depth <= 0.0 and not through:
        return shape
    island_profiles = [
        island for island in list(island_profiles or [])
        if _profile_bounds_overlap(outer_profile, island)
    ]
    if not island_profiles:
        return shape
    if face_selector in {">Z", "<Z"}:
        return _cut_profile_region_around_islands_boolean(
            shape,
            outer_profile=outer_profile,
            island_profiles=island_profiles,
            depth=depth,
            face_selector=face_selector,
            through=through,
        )
    wp = _draw_profile(shape.faces(face_selector).workplane(), outer_profile)
    for island_profile in island_profiles:
        wp = _draw_profile(wp, island_profile)
    return wp.cutThruAll() if through else wp.cutBlind(-depth)


def _profile_bounds_overlap(a, b) -> bool:
    ax0, ay0, ax1, ay1 = _profile_bounds(a)
    bx0, by0, bx1, by1 = _profile_bounds(b)
    return ax0 < bx1 and ax1 > bx0 and ay0 < by1 and ay1 > by0


def _cut_profile_region_around_islands_boolean(
    shape: "cq.Workplane",
    *,
    outer_profile,
    island_profiles,
    depth: float,
    face_selector: str = ">Z",
    through: bool = False,
) -> "cq.Workplane":
    """Boolean retained-island cut for top/bottom setups.

    Building an explicit cutting solid avoids nested-sketch ambiguity for
    rotated rectangular ribs and multiple retained islands.
    """
    bb = shape.val().BoundingBox()
    cut_depth = (bb.zmax - bb.zmin + 2.0) if through else depth
    if cut_depth <= 0.0:
        return shape
    if face_selector == "<Z":
        z_min = bb.zmin - (1.0 if through else 0.0)
    else:
        z_min = bb.zmax - cut_depth + (1.0 if through else 0.0)

    cutting = _profile_prism(outer_profile, cut_depth, z_min)
    for island_profile in island_profiles:
        island = _profile_prism(island_profile, cut_depth + 0.02, z_min - 0.01)
        cutting = cutting.cut(island)
    return shape.cut(cutting)


def _cut_profile_region_around_islands_z_band(
    shape: "cq.Workplane",
    *,
    outer_profile,
    island_profiles,
    base_height: float,
    height: float,
) -> "cq.Workplane":
    """Cut a retained-island band measured up from the stock bottom."""
    if height <= 0.0:
        return shape
    bb = shape.val().BoundingBox()
    z_min = bb.zmin + float(base_height)
    z_max = min(z_min + float(height), bb.zmax)
    if z_max <= z_min:
        return shape
    cutting = _profile_prism(outer_profile, z_max - z_min, z_min)
    for island_profile in island_profiles:
        island = _profile_prism(island_profile, (z_max - z_min) + 0.02, z_min - 0.01)
        cutting = cutting.cut(island)
    return shape.cut(cutting)


def _profile_prism(profile, depth: float, z_min: float) -> "cq.Workplane":
    prism = _draw_profile(cq.Workplane("XY"), profile).extrude(depth)
    return prism.translate((0.0, 0.0, z_min))


def machine_around_cylinder_cut(
    shape: "cq.Workplane",
    diameter: float,
    height: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    face_selector: str = ">Z",
    margin: float = 5.0,
    base_height: float | None = None,
) -> "cq.Workplane":
    """Remove a frame around a retained cylindrical boss."""
    profile = {"type": "circle", "diameter": diameter, "cx": cx, "cy": cy}
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    if base_height is not None:
        outer_profile = _stock_envelope_profile(shape, None, margin=margin)
        return _cut_profile_region_around_islands_z_band(
            shape,
            outer_profile=outer_profile,
            island_profiles=[profile],
            base_height=base_height,
            height=height,
        )
    outer = diameter + 2.0 * margin
    wp = shape.faces(face_selector).workplane().center(cx, cy).rect(outer, outer)
    wp = _draw_profile(wp, profile)
    return wp.cutBlind(-height)


def _edge_selection(shape: "cq.Workplane", selector, *, face_selector: str = ">Z") -> "cq.Workplane":
    """Resolve lightweight edge selectors to a CadQuery edge selection."""
    if isinstance(selector, str) and selector.lower() in {"all_edges", "global", "*"}:
        return shape.edges()
    if selector in (None, "all"):
        return shape.faces(face_selector).edges()
    if isinstance(selector, str):
        if selector.startswith("|"):
            return shape.edges(selector)
        if selector in {">X", "<X", ">Y", "<Y", ">Z", "<Z"}:
            return shape.faces(selector).edges()
        return shape.faces(face_selector).edges(selector)
    if isinstance(selector, dict):
        face = selector.get("face") or selector.get("face_selector") or face_selector
        edges = shape.faces(face).edges()
        direction = selector.get("edge_direction") or selector.get("direction")
        if direction:
            token = str(direction)
            if token.upper() in {"X", "Y", "Z"}:
                token = f"|{token.upper()}"
            edges = shape.faces(face).edges(token)
        bbox = selector.get("bbox") or selector.get("box")
        if bbox:
            try:
                from cadquery import selectors as cq_selectors
                return edges.filter(cq_selectors.BoxSelector(tuple(bbox[0]), tuple(bbox[1])))
            except Exception:
                return edges
        return edges
    return shape.faces(face_selector).edges()


def chamfer_edges(shape: "cq.Workplane", width: float, *, face_selector: str = ">Z", selector=None) -> "cq.Workplane":
    """Apply chamfer to top edges."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    return _edge_selection(shape, selector, face_selector=face_selector).chamfer(width)


def fillet_edges(shape: "cq.Workplane", radius: float, *, face_selector: str = ">Z", selector=None) -> "cq.Workplane":
    """Apply fillet to selected edges."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    return _edge_selection(shape, selector, face_selector=face_selector).fillet(radius)


# =============================================================================
#  Export / Import
# =============================================================================

def export_step(shape: "cq.Workplane", path: str) -> None:
    """Export shape to STEP file."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    cq.exporters.export(shape, path)


def export_stl(shape: "cq.Workplane", path: str) -> None:
    """Export shape to STL file."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    cq.exporters.export(shape, path)


def to_mesh(shape: "cq.Workplane"):
    """Convert CadQuery shape to trimesh.Trimesh using tessellation.

    Returns None if trimesh is not available.
    """
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    if not _HAS_TRIMESH:
        return None

    verts_raw, faces = shape.val().tessellate(0.1)
    # Convert CadQuery Vector objects to plain (x, y, z) tuples
    verts = [(v.x, v.y, v.z) if hasattr(v, "x") else tuple(v) for v in verts_raw]
    return trimesh.Trimesh(vertices=verts, faces=faces)


def compute_bounding_box(shape: "cq.Workplane") -> dict:
    """Compute the bounding box of the shape.

    Returns dict with keys: x_min, x_max, y_min, y_max, z_min, z_max,
    length, width, height.
    """
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")

    bb = shape.val().BoundingBox()
    return {
        "x_min": bb.xmin,
        "x_max": bb.xmax,
        "y_min": bb.ymin,
        "y_max": bb.ymax,
        "z_min": bb.zmin,
        "z_max": bb.zmax,
        "length": bb.xmax - bb.xmin,
        "width": bb.ymax - bb.ymin,
        "height": bb.zmax - bb.zmin,
    }


def compute_volume(shape: "cq.Workplane") -> float:
    """Compute volume in mm^3. Returns 0.0 if not available."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    try:
        return shape.val().Volume()
    except Exception:
        return 0.0


def import_step(path: str) -> "cq.Workplane":
    """Import shape from STEP file."""
    if not _HAS_CADQUERY:
        raise RuntimeError("cadquery is not available")
    return cq.importers.importStep(path)


# =============================================================================
#  Self-test
# =============================================================================

if __name__ == "__main__":
    if not _HAS_CADQUERY:
        print("CadQuery not available -- skipping self-tests.")
        import sys

        sys.exit(0)

    print("Running SubCAD geometry self-tests...")
    passed = 0
    failed = 0

    # ------------------------------------------------------------------
    # Test 1: create_rectangular_stock creates correct dimensions
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        bb = compute_bounding_box(stock)
        assert abs(bb["length"] - 100) < 0.01, f"Length: {bb['length']}"
        assert abs(bb["width"] - 60) < 0.01, f"Width: {bb['width']}"
        assert abs(bb["height"] - 20) < 0.01, f"Height: {bb['height']}"
        vol = compute_volume(stock)
        assert abs(vol - 100 * 60 * 20) < 0.1, f"Volume: {vol}"
        print("  PASS: create_rectangular_stock")
        passed += 1
    except Exception as e:
        print(f"  FAIL: create_rectangular_stock -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 2: drill_hole creates a through hole (volume decreases)
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        drilled = drill_hole(stock, 0, 0, 10, depth=0, through=True)
        vol_before = compute_volume(stock)
        vol_after = compute_volume(drilled)
        assert vol_after < vol_before, (
            f"Volume did not decrease: {vol_before} -> {vol_after}"
        )
        print("  PASS: drill_hole (through)")
        passed += 1
    except Exception as e:
        print(f"  FAIL: drill_hole (through) -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 3: drill_hole creates a blind hole (volume decreases)
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        drilled = drill_hole(stock, 0, 0, 10, depth=10, through=False)
        vol_before = compute_volume(stock)
        vol_after = compute_volume(drilled)
        assert vol_after < vol_before, (
            f"Volume did not decrease: {vol_before} -> {vol_after}"
        )
        print("  PASS: drill_hole (blind)")
        passed += 1
    except Exception as e:
        print(f"  FAIL: drill_hole (blind) -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 4: rectangular_pocket_cut creates a pocket
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        pocketed = rectangular_pocket_cut(stock, 10, 10, 30, 20, 5)
        vol_after = compute_volume(pocketed)
        expected_vol = 100 * 60 * 20 - 30 * 20 * 5
        assert abs(vol_after - expected_vol) < 0.5, (
            f"Volume: {vol_after} vs expected {expected_vol}"
        )
        print("  PASS: rectangular_pocket_cut")
        passed += 1
    except Exception as e:
        print(f"  FAIL: rectangular_pocket_cut -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 5: export_step writes a valid file
    # ------------------------------------------------------------------
    try:
        import os
        import tempfile

        stock = create_rectangular_stock(100, 60, 20)
        fd, tmp_path = tempfile.mkstemp(suffix=".step")
        os.close(fd)
        export_step(stock, tmp_path)
        file_size = os.path.getsize(tmp_path)
        assert file_size > 0, "STEP file is empty"
        os.unlink(tmp_path)
        print("  PASS: export_step")
        passed += 1
    except Exception as e:
        print(f"  FAIL: export_step -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 6: export_stl writes a valid file
    # ------------------------------------------------------------------
    try:
        import os
        import tempfile

        stock = create_rectangular_stock(100, 60, 20)
        fd, tmp_path = tempfile.mkstemp(suffix=".stl")
        os.close(fd)
        export_stl(stock, tmp_path)
        file_size = os.path.getsize(tmp_path)
        assert file_size > 0, "STL file is empty"
        os.unlink(tmp_path)
        print("  PASS: export_stl")
        passed += 1
    except Exception as e:
        print(f"  FAIL: export_stl -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 7: compute_volume returns reasonable value
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(50, 40, 30)
        vol = compute_volume(stock)
        assert vol > 0, "Volume should be positive"
        assert abs(vol - 60000) < 0.1, f"Expected 60000, got {vol}"
        print("  PASS: compute_volume")
        passed += 1
    except Exception as e:
        print(f"  FAIL: compute_volume -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 8: to_mesh handles null case gracefully
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        mesh = to_mesh(stock)
        if _HAS_TRIMESH:
            assert mesh is not None, "Expected mesh object"
            assert hasattr(mesh, "vertices"), "Mesh should have vertices"
            print("  PASS: to_mesh (with trimesh)")
        else:
            assert mesh is None, "Expected None without trimesh"
            print("  PASS: to_mesh (null case)")
        passed += 1
    except Exception as e:
        print(f"  FAIL: to_mesh -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 9: face_mill_cut reduces height
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        faced = face_mill_cut(stock, 5)
        bb = compute_bounding_box(faced)
        assert abs(bb["height"] - 15) < 0.01, (
            f"Height after face mill: {bb['height']}"
        )
        print("  PASS: face_mill_cut")
        passed += 1
    except Exception as e:
        print(f"  FAIL: face_mill_cut -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 10: circular_pocket_cut
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        cp = circular_pocket_cut(stock, 0, 0, 30, 10)
        vol = compute_volume(cp)
        expected = 100 * 60 * 20 - math.pi * (15**2) * 10
        assert abs(vol - expected) < 5.0, f"Volume: {vol} vs expected {expected}"
        print("  PASS: circular_pocket_cut")
        passed += 1
    except Exception as e:
        print(f"  FAIL: circular_pocket_cut -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 11: slot_cut reduces volume
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        slotted = slot_cut(stock, 20, 0, 30, 8, 10)
        vol_after = compute_volume(slotted)
        assert vol_after < compute_volume(stock), "Volume should decrease"
        print("  PASS: slot_cut")
        passed += 1
    except Exception as e:
        print(f"  FAIL: slot_cut -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 12: slot_cut through
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        slotted = slot_cut(stock, 0, 0, 30, 8, 0, through=True)
        vol_after = compute_volume(slotted)
        assert vol_after < compute_volume(stock), "Through slot should reduce volume"
        print("  PASS: slot_cut (through)")
        passed += 1
    except Exception as e:
        print(f"  FAIL: slot_cut (through) -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 13: slot_cut with angle
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        slotted = slot_cut(stock, 20, 0, 30, 8, 10, angle=45)
        vol_after = compute_volume(slotted)
        assert vol_after < compute_volume(stock), "Angled slot should reduce volume"
        print("  PASS: slot_cut (angled)")
        passed += 1
    except Exception as e:
        print(f"  FAIL: slot_cut (angled) -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 14: chamfer_edges reduces volume
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        chamfered = chamfer_edges(stock, 2.0)
        vol_chamfered = compute_volume(chamfered)
        assert vol_chamfered < compute_volume(stock), (
            "Chamfer should reduce volume"
        )
        print("  PASS: chamfer_edges")
        passed += 1
    except Exception as e:
        print(f"  FAIL: chamfer_edges -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 15: fillet_edges reduces volume
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        filleted = fillet_edges(stock, 2.0)
        vol_filleted = compute_volume(filleted)
        assert vol_filleted < compute_volume(stock), (
            "Fillet should reduce volume"
        )
        print("  PASS: fillet_edges")
        passed += 1
    except Exception as e:
        print(f"  FAIL: fillet_edges -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 16: create_cylindrical_stock
    # ------------------------------------------------------------------
    try:
        cyl = create_cylindrical_stock(50, 30)
        bb = compute_bounding_box(cyl)
        assert abs(bb["height"] - 30) < 0.01, f"Height: {bb['height']}"
        vol = compute_volume(cyl)
        expected_vol = math.pi * (25**2) * 30
        assert abs(vol - expected_vol) < 1.0, f"Volume: {vol} vs expected {expected_vol}"
        print("  PASS: create_cylindrical_stock")
        passed += 1
    except Exception as e:
        print(f"  FAIL: create_cylindrical_stock -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 16b: create_tapered_cylinder accepts equal diameters as cylinder
    # ------------------------------------------------------------------
    try:
        straight = create_tapered_cylinder(50, 50, 30)
        vol = compute_volume(straight)
        expected_vol = math.pi * (25**2) * 30
        assert abs(vol - expected_vol) < 1.0, f"Volume: {vol} vs expected {expected_vol}"
        print("  PASS: create_tapered_cylinder (equal diameters)")
        passed += 1
    except Exception as e:
        print(f"  FAIL: create_tapered_cylinder (equal diameters) -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 17: contour_cut_outer reduces volume
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        contoured = contour_cut_outer(stock, 5)
        vol_after = compute_volume(contoured)
        assert vol_after < compute_volume(stock), (
            "Contour should reduce volume"
        )
        print("  PASS: contour_cut_outer")
        passed += 1
    except Exception as e:
        print(f"  FAIL: contour_cut_outer -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 18: contour_cut_outer with stepdown
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        contoured = contour_cut_outer(stock, 6, stepdown=2.0)
        vol_after = compute_volume(contoured)
        assert vol_after < compute_volume(stock), (
            "Multi-pass contour should reduce volume"
        )
        print("  PASS: contour_cut_outer (stepdown)")
        passed += 1
    except Exception as e:
        print(f"  FAIL: contour_cut_outer (stepdown) -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 19: rectangular_pocket_cut with corner_radius
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        pocketed = rectangular_pocket_cut(stock, 10, 10, 30, 20, 5, corner_radius=3.0)
        vol_after = compute_volume(pocketed)
        expected_max = 100 * 60 * 20 - (30 - 2 * 3) * (20 - 2 * 3) * 5
        # Volume should decrease (pocket removes material)
        assert vol_after < compute_volume(stock), "Radius pocket should reduce volume"
        print("  PASS: rectangular_pocket_cut (corner_radius)")
        passed += 1
    except Exception as e:
        print(f"  FAIL: rectangular_pocket_cut (corner_radius) -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 20: compute_bounding_box returns correct keys
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        bb = compute_bounding_box(stock)
        required_keys = {
            "x_min", "x_max", "y_min", "y_max", "z_min", "z_max",
            "length", "width", "height",
        }
        assert required_keys.issubset(bb.keys()), (
            f"Missing keys: {required_keys - set(bb.keys())}"
        )
        print("  PASS: compute_bounding_box keys")
        passed += 1
    except Exception as e:
        print(f"  FAIL: compute_bounding_box keys -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 21: import_step (null / graceful handling)
    # ------------------------------------------------------------------
    try:
        # import_step requires an actual file, so just verify the function exists
        # and accepts a string argument.  We don't test actual import since no
        # STEP file is available in the test environment.
        assert callable(import_step), "import_step should be callable"
        print("  PASS: import_step (callable)")
        passed += 1
    except Exception as e:
        print(f"  FAIL: import_step (callable) -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 22: face_selector multi-face support
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        # Drill from bottom face
        drilled = drill_hole(stock, 0, 0, 10, depth=10, through=False,
                            face_selector="<Z")
        vol_after = compute_volume(drilled)
        assert vol_after < compute_volume(stock), "Bottom-face drill should reduce volume"
        print("  PASS: drill_hole (bottom face <Z)")
        passed += 1
    except Exception as e:
        print(f"  FAIL: drill_hole (bottom face <Z) -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 23: face_mill_cut from bottom face reduces height
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        faced = face_mill_cut(stock, 5, face_selector="<Z")
        bb = compute_bounding_box(faced)
        assert abs(bb["height"] - 15) < 0.01, (
            f"Height after bottom face_mill: {bb['height']}"
        )
        print("  PASS: face_mill_cut (bottom face <Z)")
        passed += 1
    except Exception as e:
        print(f"  FAIL: face_mill_cut (bottom face <Z) -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Test 24: chamfer_edges from bottom face
    # ------------------------------------------------------------------
    try:
        stock = create_rectangular_stock(100, 60, 20)
        chamfered = chamfer_edges(stock, 2.0, face_selector="<Z")
        vol_chamfered = compute_volume(chamfered)
        assert vol_chamfered < compute_volume(stock), (
            "Bottom chamfer should reduce volume"
        )
        print("  PASS: chamfer_edges (bottom face <Z)")
        passed += 1
    except Exception as e:
        print(f"  FAIL: chamfer_edges (bottom face <Z) -- {e}")
        failed += 1

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    total = passed + failed
    print(f"\n{'=' * 50}")
    print(f"Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed > 0:
        import sys

        sys.exit(1)
