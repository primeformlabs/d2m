"""Tests for pure SubCAD operation families used by STEP coverage."""

import math

from src.subcad import Stock, create_operation
from src.data.pure_subcad_planner import build_deterministic_subcad_code, plan_pure_subcad_features
from src.data.subcad_repl import run_subcad


def check(cond, label):
    if not cond:
        raise AssertionError(label)
    print(f"  PASS  {label}")


print("1. Pure planner maps STEP/CadQuery feature families ...")
plan = plan_pure_subcad_features(
    [
        {"op_name": "chamfer"},
        {"op_name": "fillet"},
        {"op_name": "union"},
        {"op_name": "circle"},
        {"op_name": "extrude"},
        {"op_name": "revolve"},
        {"op_name": "shell", "function": "part.faces.shell"},
        {"op_name": "sweep"},
        {"op_name": "loft"},
        {"op_name": "polygon"},
        {"op_name": "rarray"},
    ],
    '.circle(10).extrude(5).union(x).revolve().faces(">Z").shell(1).sweep(p).loft()',
)
ops = {feature.operation for feature in plan.features}
check(plan.compatible, "planner considers mapped pure operations compatible")
check("edge_chamfer" in ops, "planner maps chamfer to edge_chamfer")
check("edge_fillet" in ops, "planner maps fillet to edge_fillet")
check("machine_around_profile" in ops, "planner maps union to retained-material operation")
check("turn_profile" in ops, "planner maps revolve to turn_profile")
check("thin_wall_pocket" in ops, "planner maps shell to thin_wall_pocket")
check("sweep_mill" in ops, "planner maps sweep to sweep_mill")
check("loft_mill" in ops, "planner maps loft to loft_mill")

tapered_plan = plan_pure_subcad_features(
    [{"op_name": "circle"}, {"op_name": "extrude", "keywords": [{"arg": "taper", "value": "-5"}]}],
    ".circle(40).extrude(10, taper=-5)",
)
check(
    any(feature.operation == "turn_profile" and feature.family == "axisymmetric"
        for feature in tapered_plan.features),
    "planner maps tapered circle extrude to turn_profile",
)

closed_shell = plan_pure_subcad_features(
    [{"op_name": "box"}, {"op_name": "shell", "function": "base.shell"}],
    ".box(10, 10, 10).shell(-1)",
)
check(not closed_shell.compatible, "planner rejects closed inaccessible shells")
check(
    any(feature.planner_status == "unsupported_unmachinable" for feature in closed_shell.features),
    "closed shell is labeled unsupported_unmachinable",
)

side_gusset = plan_pure_subcad_features(
    [
        {"op_name": "box"},
        {"op_name": "faces"},
        {"op_name": "workplane"},
        {"op_name": "polyline"},
        {"op_name": "extrude"},
    ],
    """
    cq.Workplane("XY")
      .box(40, 30, 10)
      .faces(">X").workplane()
      .polyline([(0, 0), (12, 0), (0, 8)])
      .close()
      .extrude(4)
    """,
)
check(not side_gusset.compatible, "planner rejects side-face additive gussets")
check(
    any(feature.planner_status == "needs_manual_review"
        and feature.operation == "oriented_retained_material"
        for feature in side_gusset.features),
    "side-face additive gusset is labeled needs_manual_review",
)

bottom_cut = plan_pure_subcad_features(
    [
        {"op_name": "box"},
        {"op_name": "faces"},
        {"op_name": "workplane"},
        {"op_name": "rect"},
        {"op_name": "cutBlind"},
    ],
    """
    cq.Workplane("XY")
      .box(40, 30, 10)
      .faces("<Z").workplane()
      .rect(8, 12)
      .cutBlind(2)
    """,
)
check(bottom_cut.compatible, "planner preserves bottom-face cutBlind compatibility")
check(
    not any(feature.operation == "oriented_retained_material" for feature in bottom_cut.features),
    "bottom-face cutBlind is not treated as retained additive material",
)

top_rib_extrude = plan_pure_subcad_features(
    [
        {"op_name": "box"},
        {"op_name": "faces"},
        {"op_name": "workplane"},
        {"op_name": "rect"},
        {"op_name": "extrude"},
    ],
    """
    cq.Workplane("XY")
      .box(40, 30, 5)
      .faces(">Z").workplane()
      .rect(8, 20)
      .extrude(4)
    """,
)
check(top_rib_extrude.compatible, "planner accepts top-face additive retained material")
check(
    any(feature.operation == "machine_around_profile"
        and feature.family == "retained_material"
        for feature in top_rib_extrude.features),
    "top-face additive extrude maps to retained-material operation",
)

retained_pattern_plan = plan_pure_subcad_features(
    [],
    """
    flange_thickness = 8.0
    fusion_overlap = 0.5
    rib_count = 3
    rib_spacing = 12.0
    rib_width = 6.0
    rib_height = 4.0
    boss_height = 12.0
    boss_diameter = 20.0
    boss = (
        cq.Workplane("XY")
        .center(10.0, 10.0)
        .circle(boss_diameter / 2.0)
        .extrude(boss_height)
        .translate((0, 0, flange_thickness - fusion_overlap))
    )
    result = result.union(boss)
    for i in range(rib_count):
        rib_x = -4.0 + i * rib_spacing
        rib = (
            cq.Workplane("XY")
            .center(rib_x, 2.0)
            .rect(rib_spacing * 0.8, rib_width)
            .extrude(rib_height)
            .translate((0, 0, flange_thickness - fusion_overlap))
        )
        result = result.union(rib)
    """,
)
pattern_evidence = [
    feature.evidence for feature in retained_pattern_plan.features
    if feature.operation == "machine_around_profiles"
    and feature.evidence.get("suggested_subcad")
]
check(bool(pattern_evidence), "planner extracts multi-rib retained-pattern evidence")
check(
    [round(profile["cx"], 6) for profile in pattern_evidence[0]["profiles"]] == [-4.0, 8.0, 20.0],
    "retained-pattern evidence preserves loop-derived rib centers",
)
level_evidence = [
    feature.evidence for feature in retained_pattern_plan.features
    if feature.evidence.get("suggested_sequence")
]
check(bool(level_evidence), "planner suggests mixed-height retained operation sequence")
check(
    "base_height=11.5" in level_evidence[0]["suggested_subcad"]
    and "base_height=7.5" in level_evidence[0]["suggested_subcad"],
    "mixed-height retained sequence preserves upper and lower z bands",
)

base_cut_plan = plan_pure_subcad_features(
    [],
    """
    flange_leg_length = 80.0
    flange_leg_width = 30.0
    flange_thickness = 8.0
    inner_cut_width = 20.0
    cut = (
        cq.Workplane('XY')
        .rect(flange_leg_length - inner_cut_width, flange_leg_width - inner_cut_width)
        .translate((inner_cut_width / 2.0, inner_cut_width / 2.0))
        .extrude(flange_thickness)
    )
    l_shape = base.cut(cut)
    """,
)
base_cut_evidence = [
    feature.evidence for feature in base_cut_plan.features
    if feature.operation == "pocket" and feature.evidence.get("suggested_subcad")
]
check(bool(base_cut_evidence), "planner extracts base-band rectangular cut evidence")
check("base_height=0.0" in base_cut_evidence[0]["suggested_subcad"],
      "base-band rectangular cut suggests z-band pocket")

layered_part_plan = plan_pure_subcad_features(
    [],
    """
    flange_leg_length = 80.0
    flange_leg_width = 30.0
    flange_thickness = 8.0
    inner_cut_width = 20.0
    boss_diameter = 20.0
    boss_height = 12.0
    rib_height = 4.0
    rib_width = 6.0
    rib_spacing = 12.0
    fusion_overlap = 0.5
    base = cq.Workplane('XY').rect(flange_leg_length, flange_leg_width).extrude(flange_thickness)
    cut = (
        cq.Workplane('XY')
        .rect(flange_leg_length - inner_cut_width, flange_leg_width - inner_cut_width)
        .translate((inner_cut_width / 2.0, inner_cut_width / 2.0))
        .extrude(flange_thickness)
    )
    l_shape = base.cut(cut)
    boss = (
        cq.Workplane('XY')
        .center(inner_cut_width / 2.0, inner_cut_width / 2.0)
        .circle(boss_diameter / 2.0)
        .extrude(boss_height)
        .translate((0, 0, flange_thickness - fusion_overlap))
    )
    result = l_shape.union(boss)
    for i in range(3):
        rib_x = -4.0 + i * rib_spacing
        rib = (
            cq.Workplane('XY')
            .center(rib_x, 2.0)
            .rect(rib_spacing * 0.8, rib_width)
            .extrude(rib_height)
            .translate((0, 0, flange_thickness - fusion_overlap))
        )
        result = result.union(rib)
    """,
)
layered_evidence = [
    feature.evidence for feature in layered_part_plan.features
    if feature.operation == "layered_retained_profile_plan"
]
check(bool(layered_evidence), "planner emits full layered retained-profile plan")
check(
    "Stock.rectangular(80, 40, 19.5)" in layered_evidence[0]["suggested_subcad"],
    "layered retained-profile plan expands stock to full boss envelope",
)
layered_with_holes_code = """
flange_leg_length = 80.0
flange_leg_width = 30.0
flange_thickness = 8.0
inner_cut_width = 20.0
boss_diameter = 20.0
boss_height = 12.0
rib_height = 4.0
rib_width = 6.0
rib_spacing = 12.0
fusion_overlap = 0.5
hole_diameter = 5.5
hole_positions = [(-20.0, 5.0), (20.0, 5.0), (30.0, -20.0), (30.0, 20.0)]
base = cq.Workplane('XY').rect(flange_leg_length, flange_leg_width).extrude(flange_thickness)
cut = (
    cq.Workplane('XY')
    .rect(flange_leg_length - inner_cut_width, flange_leg_width - inner_cut_width)
    .translate((inner_cut_width / 2.0, inner_cut_width / 2.0))
    .extrude(flange_thickness)
)
l_shape = base.cut(cut)
boss = (
    cq.Workplane('XY')
    .center(inner_cut_width / 2.0, inner_cut_width / 2.0)
    .circle(boss_diameter / 2.0)
    .extrude(boss_height)
    .translate((0, 0, flange_thickness - fusion_overlap))
)
result = l_shape.union(boss)
for i in range(3):
    rib_x = -4.0 + i * rib_spacing
    rib = (
        cq.Workplane('XY')
        .center(rib_x, 2.0)
        .rect(rib_spacing * 0.8, rib_width)
        .extrude(rib_height)
        .translate((0, 0, flange_thickness - fusion_overlap))
    )
    result = result.union(rib)
for (x, y) in hole_positions:
    result = result.faces('>Z').workplane().center(x, y).hole(hole_diameter)
"""
deterministic_code = build_deterministic_subcad_code(layered_with_holes_code, [])
check(deterministic_code is not None, "planner builds deterministic layered-retained SubCAD code")
check(
    ".drill(5.5, through=True, cx=30, cy=20)" in deterministic_code,
    "deterministic layered-retained code includes source-derived drill positions",
)
deterministic_exec = run_subcad(deterministic_code)
check(deterministic_exec["success"], "deterministic layered-retained SubCAD code executes")

box_chamfer_code = "result = cq.Workplane('XY').box(20,20,2).faces('>Z').edges().chamfer(1,1)"
box_chamfer_subcad = build_deterministic_subcad_code(box_chamfer_code, [])
check(box_chamfer_subcad is not None, "planner builds deterministic box-chamfer SubCAD code")
check(
    "Stock.rectangular(20, 20, 2)" in box_chamfer_subcad
    and '.edge_chamfer(">Z", width=1)' in box_chamfer_subcad,
    "deterministic box-chamfer code preserves dimensions and top chamfer",
)
box_chamfer_exec = run_subcad(box_chamfer_subcad)
check(box_chamfer_exec["success"], "deterministic box-chamfer SubCAD code executes")

box_hole_code = "result = cq.Workplane('XY').box(50, 30, 5).hole(5)"
box_hole_subcad = build_deterministic_subcad_code(box_hole_code, [])
check(box_hole_subcad is not None, "planner builds deterministic box-hole SubCAD code")
check(
    "Stock.rectangular(50, 30, 5)" in box_hole_subcad
    and ".drill(5, through=True, cx=0.0, cy=0.0)" in box_hole_subcad,
    "deterministic box-hole code preserves stock and center hole",
)
box_hole_exec = run_subcad(box_hole_subcad)
check(box_hole_exec["success"], "deterministic box-hole SubCAD code executes")

box_counterbore_code = "result = cq.Workplane('XY').box(10,10,2).faces('>Z').workplane().cboreHole(4,6,2,1)"
box_counterbore_subcad = build_deterministic_subcad_code(box_counterbore_code, [])
check(box_counterbore_subcad is not None, "planner builds deterministic box-counterbore SubCAD code")
check(
    "Stock.rectangular(10, 10, 2)" in box_counterbore_subcad
    and ".counterbore(4, 6, 2, through=False, cx=0.0, cy=0.0)" in box_counterbore_subcad,
    "deterministic box-counterbore code preserves stock and counterbore",
)
box_counterbore_exec = run_subcad(box_counterbore_subcad)
check(box_counterbore_exec["success"], "deterministic box-counterbore SubCAD code executes")

box_pocket_code = """
import cadquery as cq
from types import SimpleNamespace as Measures
m = Measures(plate_width=80.0, plate_depth=60.0, plate_thickness=6.0, chamfer=1.5, pocket_width=40.0, pocket_depth=30.0, pocket_cut_depth=3.0)
result = (cq.Workplane('XY')
    .box(m.plate_width, m.plate_depth, m.plate_thickness)
    .translate((0,0,m.plate_thickness/2))
    .edges().chamfer(m.chamfer)
    .faces('>Z').workplane()
    .rect(m.pocket_width, m.pocket_depth)
    .cutBlind(m.pocket_cut_depth)
)
"""
box_pocket_subcad = build_deterministic_subcad_code(box_pocket_code, [])
check(box_pocket_subcad is not None, "planner builds deterministic box-pocket SubCAD code")
check(
    "Stock.rectangular(80, 60, 6)" in box_pocket_subcad
    and '.edge_chamfer("all_edges", width=1.5)' in box_pocket_subcad
    and ".pocket(" not in box_pocket_subcad,
    "deterministic box-pocket code preserves SimpleNamespace dimensions and omits positive no-op cutBlind",
)
box_pocket_exec = run_subcad(box_pocket_subcad)
check(box_pocket_exec["success"], "deterministic box-pocket SubCAD code executes")

real_box_pocket_subcad = build_deterministic_subcad_code(
    box_pocket_code.replace("cutBlind(m.pocket_cut_depth)", "cutBlind(-m.pocket_cut_depth)"),
    [],
)
check(real_box_pocket_subcad is not None, "planner builds deterministic negative box-pocket SubCAD code")
check(
    ".pocket(width=30, length=40, depth=3" in real_box_pocket_subcad,
    "deterministic box-pocket code emits negative cutBlind as a real pocket",
)
real_box_pocket_exec = run_subcad(real_box_pocket_subcad)
check(real_box_pocket_exec["success"], "deterministic negative box-pocket SubCAD code executes")

side_counterbore_profile_code = """
import cadquery as cq
vertical_leg_height = 70.0
vertical_leg_thickness = 12.0
horizontal_leg_length = 100.0
horizontal_leg_thickness = 12.0
bracket_thickness = 8.0
hole_clearance = 5.5
cbore_diameter = 7.0
cbore_depth = 2.0
hole_spacing = 20.0
hole_margin = 10.0
edge_chamfer = 1.0
profile = (
    cq.Workplane("XY")
    .moveTo(0, 0)
    .lineTo(vertical_leg_thickness, 0)
    .lineTo(vertical_leg_thickness, vertical_leg_height - horizontal_leg_thickness)
    .lineTo(horizontal_leg_length, vertical_leg_height - horizontal_leg_thickness)
    .lineTo(horizontal_leg_length, vertical_leg_height)
    .lineTo(0, vertical_leg_height)
    .close()
    .extrude(bracket_thickness)
)
profile = profile.edges("|Z and >X").chamfer(edge_chamfer)
usable_y_start = vertical_leg_thickness + hole_margin
usable_y_end = horizontal_leg_length - hole_margin
num_holes = int((usable_y_end - usable_y_start) / hole_spacing) + 1
hole_positions = []
for i in range(num_holes):
    y_pos = usable_y_start + i * hole_spacing
    z_pos = bracket_thickness / 2
    hole_positions.append((y_pos, z_pos))
result = (
    profile.faces(">X").workplane()
    .pushPoints(hole_positions)
    .cboreHole(hole_clearance, cbore_diameter, cbore_depth)
)
"""
side_counterbore_profile_subcad = build_deterministic_subcad_code(side_counterbore_profile_code, [])
check(side_counterbore_profile_subcad is not None, "planner builds deterministic side-counterbore profile code")
check(
    side_counterbore_profile_subcad.count('.counterbore(5.5, 7, 2, through=True') == 4
    and 'face_selector="<X"' in side_counterbore_profile_subcad,
    "deterministic side-counterbore profile code maps loop-built pushPoints to side setup",
)
side_counterbore_profile_exec = run_subcad(side_counterbore_profile_subcad)
check(side_counterbore_profile_exec["success"], "deterministic side-counterbore profile SubCAD code executes")

rect_plate_code = """
import cadquery as cq
plate_width = 80.0
plate_depth = 60.0
plate_thickness = 4.0
rib_width = 5.0
rib_height = 2.0
hole_diameter = 6.0
hole_spacing_x = 20.0
hole_spacing_y = 25.0
num_holes_x = 4
num_holes_y = 2
central_pocket_width = 20.0
central_pocket_depth = 15.0
central_pocket_cut = plate_thickness * 0.6
chamfer_size = 0.5
result = (
    cq.Workplane("XY")
    .rect(plate_width, plate_depth)
    .extrude(plate_thickness)
    .faces("+Z").workplane()
    .rect(central_pocket_width, central_pocket_depth)
    .cutBlind(central_pocket_cut)
    .faces("+Z").workplane()
    .rect(plate_width + 2 * rib_width, plate_depth + 2 * rib_width)
    .extrude(rib_height)
    .faces("+Z").workplane()
    .rect(plate_width, plate_depth)
    .cutBlind(rib_height)
    .faces("+Z").edges()
    .chamfer(chamfer_size)
    .faces("+Z").workplane()
    .rarray(hole_spacing_x, hole_spacing_y, num_holes_x, num_holes_y, True)
    .hole(hole_diameter)
)
"""
rect_plate_subcad = build_deterministic_subcad_code(rect_plate_code, [])
check(rect_plate_subcad is not None, "planner builds deterministic raised rectangular plate code")
check(
    "Stock.rectangular(90, 70, 6)" in rect_plate_subcad
    and ".machine_around_profile({'type': 'rect', 'length': 80" in rect_plate_subcad
    and rect_plate_subcad.count(".drill(6, through=True") == 8,
    "deterministic raised rectangular plate code preserves frame and rarray holes",
)
check(
    "central_pocket" not in rect_plate_subcad and "pocket(" not in rect_plate_subcad,
    "deterministic raised rectangular plate code omits no-op positive cutBlind pockets",
)
rect_plate_exec = run_subcad(rect_plate_subcad)
check(rect_plate_exec["success"], "deterministic raised rectangular plate SubCAD code executes")

l_bracket_code = """
class LBracket:
    def __init__(self, workplane, measures):
        self.model = workplane
        self.m = measures
        self.build()
    def build(self):
        m = self.m
        bracket = (
            cq.Workplane("XY")
            .moveTo(0, 0)
            .lineTo(m.horizontal_leg_length, 0)
            .lineTo(m.horizontal_leg_length, m.leg_width)
            .lineTo(m.leg_width, m.leg_width)
            .lineTo(m.leg_width, m.vertical_leg_length)
            .lineTo(0, m.vertical_leg_length)
            .close()
            .extrude(m.thickness)
        )
        bracket = bracket.edges("|Z").chamfer(m.edge_chamfer)
        bracket = bracket.faces(">Z").workplane().pushPoints([
            (m.horizontal_leg_length / 2 - m.hole_spacing / 2, m.leg_width),
            (m.horizontal_leg_length / 2 + m.hole_spacing / 2, m.leg_width),
        ]).cboreHole(m.hole_diameter, m.countersink_diameter, m.countersink_depth)
        gusset = cq.Workplane("XY").moveTo(0, 0).lineTo(m.gusset_base, 0).lineTo(0, m.gusset_height).close().extrude(m.thickness)
        self.model = bracket.union(gusset)
measures = Measures(
    horizontal_leg_length=60.0,
    vertical_leg_length=80.0,
    leg_width=20.0,
    thickness=8.0,
    edge_chamfer=1.0,
    hole_diameter=5.0,
    countersink_diameter=8.0,
    countersink_depth=2.0,
    hole_spacing=30.0,
    gusset_base=12.0,
    gusset_height=12.0,
)
result = LBracket(cq.Workplane("XY"), measures).model
"""
l_bracket_subcad = build_deterministic_subcad_code(l_bracket_code, [])
check(l_bracket_subcad is not None, "planner builds deterministic L-bracket SubCAD code")
check(
    "Stock.rectangular(60, 80, 8)" in l_bracket_subcad
    and l_bracket_subcad.count(".counterbore(5, 8, 2") == 2,
    "deterministic L-bracket code preserves profile and counterbores",
)
l_bracket_exec = run_subcad(l_bracket_subcad)
check(l_bracket_exec["success"], "deterministic L-bracket SubCAD code executes")

l_bracket_rarray_code = """
class LBracket:
    def __init__(self, workplane, measures):
        self.model = workplane
        self.measures = measures
        self.build()
    def build(self):
        m = self.measures
        base = (
            cq.Workplane("XY")
            .moveTo(0, 0)
            .lineTo(m.thickness, 0)
            .lineTo(m.thickness, m.leg_height)
            .lineTo(m.leg_length, m.leg_height)
            .lineTo(m.leg_length, m.leg_height + m.thickness)
            .lineTo(0, m.leg_height + m.thickness)
            .close()
            .extrude(m.thickness)
        )
        rib = cq.Workplane("XY").moveTo(m.thickness, m.leg_height).lineTo(m.thickness + m.rib_width, m.leg_height).lineTo(m.thickness, m.leg_height + m.rib_height).close().extrude(m.thickness)
        bracket = base.union(rib)
        bracket = bracket.faces(">Z").workplane().moveTo(m.thickness + m.hole_margin, m.leg_height + m.thickness / 2).rarray(m.hole_spacing, 0, 4, 1).hole(m.hole_clearance_diameter)
        bracket = bracket.edges("|Z").chamfer(m.chamfer)
        self.model = bracket
measures = Measures(
    leg_length=80.0,
    leg_height=60.0,
    thickness=8.0,
    rib_width=30.0,
    rib_height=6.0,
    hole_clearance_diameter=7.0,
    hole_margin=10.0,
    hole_spacing=17.0,
    chamfer=1.0,
)
result = LBracket(cq.Workplane("XY"), measures).model
"""
l_bracket_rarray_subcad = build_deterministic_subcad_code(l_bracket_rarray_code, [])
check(l_bracket_rarray_subcad is not None, "planner builds deterministic L-bracket rarray SubCAD code")
check(
    "Stock.rectangular(80, 68, 8)" in l_bracket_rarray_subcad
    and ".machine_around_profile({'type': 'polygon'" not in l_bracket_rarray_subcad
    and l_bracket_rarray_subcad.count(".drill(7, through=True") == 4,
    "deterministic L-bracket rarray code preserves profile, omits absorbed rib, and preserves hole row",
)
check(
    "cx=-31.5, cy=-34" in l_bracket_rarray_subcad
    and "cx=-14.5, cy=-34" in l_bracket_rarray_subcad,
    "deterministic L-bracket rarray code matches CadQuery local workplane hole positions",
)
l_bracket_rarray_exec = run_subcad(l_bracket_rarray_subcad)
check(l_bracket_rarray_exec["success"], "deterministic L-bracket rarray SubCAD code executes")

simple_l_profile_code = """
import cadquery as cq
vertical_leg_height=80.0
horizontal_leg_length=60.0
leg_width=15.0
wall_thickness=5.0
result = (
    cq.Workplane('XY')
    .moveTo(0,0)
    .lineTo(0, vertical_leg_height)
    .lineTo(leg_width, vertical_leg_height)
    .lineTo(leg_width, leg_width)
    .lineTo(horizontal_leg_length, leg_width)
    .lineTo(horizontal_leg_length, 0)
    .close()
    .extrude(wall_thickness)
)
"""
simple_l_profile_subcad = build_deterministic_subcad_code(simple_l_profile_code, [])
check(simple_l_profile_subcad is not None, "planner builds deterministic simple L-profile SubCAD code")
check(
    "Stock.rectangular(60, 80, 5)" in simple_l_profile_subcad
    and ".profile_cutout({'type': 'polygon'" in simple_l_profile_subcad,
    "deterministic simple L-profile code preserves bounding stock and profile cutout",
)
simple_l_profile_exec = run_subcad(simple_l_profile_subcad)
check(simple_l_profile_exec["success"], "deterministic simple L-profile SubCAD code executes")

variable_points_profile_code = """
import cadquery as cq
bracket_length = 80
bracket_width = 40
bracket_thickness = 10
notch_size = 8
points = [
    (-bracket_length/2, -bracket_width/2),
    (bracket_length/2, -bracket_width/2),
    (bracket_length/2, bracket_width/2 - notch_size),
    (bracket_length/2 - notch_size, bracket_width/2),
    (-bracket_length/2, bracket_width/2),
    (-bracket_length/2, -bracket_width/2)
]
result = cq.Workplane("XY").polyline(points).close().extrude(bracket_thickness)
"""
variable_points_profile_subcad = build_deterministic_subcad_code(variable_points_profile_code, [])
check(variable_points_profile_subcad is not None, "planner builds deterministic variable-points profile code")
check(
    "Stock.rectangular(80, 40, 10)" in variable_points_profile_subcad
    and "(40, 12)" in variable_points_profile_subcad
    and "(32, 20)" in variable_points_profile_subcad,
    "deterministic variable-points profile code resolves point-list expressions",
)
variable_points_profile_exec = run_subcad(variable_points_profile_subcad)
check(variable_points_profile_exec["success"], "deterministic variable-points profile SubCAD code executes")

mirrored_profile_code = """
import cadquery as cq
pts = [(0, 20), (4, 20), (4, 2), (2, 2), (2, -2), (4, -2), (4, -20), (0, -20)]
result = cq.Workplane("front").polyline(pts).close().mirrorY().extrude(80)
"""
mirrored_profile_subcad = build_deterministic_subcad_code(mirrored_profile_code, [])
check(mirrored_profile_subcad is not None, "planner builds deterministic mirrored profile code")
check(
    "Stock.rectangular(8, 40, 80)" in mirrored_profile_subcad
    and "(-4, 20)" in mirrored_profile_subcad,
    "deterministic mirrored profile code expands half-profile across Y axis",
)
mirrored_profile_exec = run_subcad(mirrored_profile_subcad)
check(mirrored_profile_exec["success"], "deterministic mirrored profile SubCAD code executes")

loop_points_profile_code = """
import cadquery as cq
base_width = 60.0
base_depth = 30.0
base_height = 10.0
fin_height = 4.0
fin_thickness = 3.0
num_fins = 5
gap = (base_width - num_fins * fin_thickness) / (num_fins + 1)
points = [(0, 0), (0, base_height)]
x = 0.0
for i in range(num_fins):
    x += gap
    points.append((x, base_height))
    points.append((x, base_height + fin_height))
    x += fin_thickness
    points.append((x, base_height + fin_height))
    points.append((x, base_height))
x += gap
points.append((base_width, base_height))
points.append((base_width, 0))
result = cq.Workplane("XZ").polyline(points).close().extrude(base_depth)
"""
loop_points_profile_subcad = build_deterministic_subcad_code(loop_points_profile_code, [])
check(loop_points_profile_subcad is not None, "planner builds deterministic loop-points profile code")
check(
    "Stock.rectangular(60, 14, 30)" in loop_points_profile_subcad
    and "(-22.5, 3)" in loop_points_profile_subcad
    and "(-19.5, 7)" in loop_points_profile_subcad,
    "deterministic loop-points profile code resolves append-built point lists",
)
loop_points_profile_exec = run_subcad(loop_points_profile_subcad)
check(loop_points_profile_exec["success"], "deterministic loop-points profile SubCAD code executes")

arc_profile_code = """
import cadquery as cq
arm_length = 80
arm_width = 30
arm_height = 15
radius_fillet = 5
w = arm_width/2
h = arm_height
r = radius_fillet
result = (
    cq.Workplane('XY')
    .moveTo(-w,0)
    .lineTo(w,0)
    .lineTo(w, h - r)
    .threePointArc((w, h), (w - r, h))
    .lineTo(-w + r, h)
    .threePointArc((-w, h), (-w, h - r))
    .close()
    .extrude(arm_length)
)
"""
arc_profile_subcad = build_deterministic_subcad_code(arc_profile_code, [])
check(arc_profile_subcad is not None, "planner builds deterministic arc-profile SubCAD code")
check(
    "Stock.rectangular(" in arc_profile_subcad
    and arc_profile_subcad.count("),") > 20,
    "deterministic arc-profile code samples three-point arcs",
)
arc_profile_exec = run_subcad(arc_profile_subcad)
check(arc_profile_exec["success"], "deterministic arc-profile SubCAD code executes")

l_bracket_top_rib_code = """
import cadquery as cq
from types import SimpleNamespace as Measures
horizontal_leg_length = 80.0
vertical_leg_length = 70.0
thickness = 10.0
vertical_thickness = thickness
rib_length = 60.0
rib_thickness = 8.0
rib_height = 12.0
clearance_hole_diameter = 12.0
chamfer_dist = 1.0
class LBracket:
    def __init__(self, workplane, measures):
        self.model = workplane
        self.measures = measures
        self.build()
    def build(self):
        m = self.measures
        base = cq.Workplane("XY").moveTo(0, 0).lineTo(m.horizontal_leg_length, 0).lineTo(m.horizontal_leg_length, m.thickness).lineTo(m.vertical_thickness, m.thickness).lineTo(m.vertical_thickness, m.vertical_leg_length + m.thickness).lineTo(0, m.vertical_leg_length + m.thickness).close().extrude(m.thickness)
        rib = base.faces(">Z").workplane().center(m.horizontal_leg_length / 2, m.thickness / 2).rect(m.rib_length, m.rib_thickness).extrude(m.rib_height)
        rib = rib.faces(">Z").workplane().hole(m.clearance_hole_diameter)
        self.model = rib.edges("|Z").chamfer(m.chamfer_dist)
measures = Measures(horizontal_leg_length=horizontal_leg_length, vertical_leg_length=vertical_leg_length, thickness=thickness, vertical_thickness=vertical_thickness, rib_length=rib_length, rib_thickness=rib_thickness, rib_height=rib_height, clearance_hole_diameter=clearance_hole_diameter, chamfer_dist=chamfer_dist)
result = LBracket(cq.Workplane("XY"), measures).model
"""
l_bracket_top_rib_subcad = build_deterministic_subcad_code(l_bracket_top_rib_code, [])
check(l_bracket_top_rib_subcad is not None, "planner builds deterministic L-bracket top-rib SubCAD code")
check(
    "Stock.rectangular(80, 80, 22)" in l_bracket_top_rib_subcad
    and "base_height=10" in l_bracket_top_rib_subcad
    and ".drill(" not in l_bracket_top_rib_subcad,
    "deterministic L-bracket top-rib code preserves z-band rib and skips no-op hole",
)
l_bracket_top_rib_exec = run_subcad(l_bracket_top_rib_subcad)
check(l_bracket_top_rib_exec["success"], "deterministic L-bracket top-rib SubCAD code executes")

tapered_l_profile_code = """
import cadquery as cq
from types import SimpleNamespace as Measures
horizontal_length = 80.0
vertical_height = 80.0
leg_thickness = 10.0
taper_length = 20.0
extrude_depth = 10.0
hole_diameter = 4.5
hole_count = 5
hole_spacing = 12.0
hole_offset_from_bottom = 15.0
chamfer_size = 1.0
notch_width = 6.0
notch_height = 12.0
notch_offset_y = 30.0
class LBracket:
    def __init__(self, workplane, measures):
        self.model = workplane
        self.measures = measures
        self.build()
    def build(self):
        m = self.measures
        base = cq.Workplane("XY").polyline([(0, 0), (m.horizontal_length - m.taper_length, 0), (m.horizontal_length, m.leg_thickness), (m.horizontal_length, m.vertical_height), (m.leg_thickness, m.vertical_height), (m.leg_thickness, m.leg_thickness), (0, m.leg_thickness), (0, 0)]).close().extrude(m.extrude_depth)
        solid = base.edges("|Z").chamfer(m.chamfer_size)
        self.model = solid
measures = Measures(horizontal_length=horizontal_length, vertical_height=vertical_height, leg_thickness=leg_thickness, taper_length=taper_length, extrude_depth=extrude_depth, hole_diameter=hole_diameter, hole_count=hole_count, hole_spacing=hole_spacing, hole_offset_from_bottom=hole_offset_from_bottom, chamfer_size=chamfer_size, notch_width=notch_width, notch_height=notch_height, notch_offset_y=notch_offset_y)
result = LBracket(cq.Workplane("XY"), measures).model
"""
tapered_l_profile_subcad = build_deterministic_subcad_code(tapered_l_profile_code, [])
check(tapered_l_profile_subcad is not None, "planner builds deterministic tapered L-profile SubCAD code")
check(
    "Stock.rectangular(80, 80, 10)" in tapered_l_profile_subcad
    and ".edge_chamfer(\"|Z\", width=1)" in tapered_l_profile_subcad
    and ".drill(" not in tapered_l_profile_subcad,
    "deterministic tapered L-profile code preserves taper and skips no-op holes",
)
tapered_l_profile_exec = run_subcad(tapered_l_profile_subcad)
check(tapered_l_profile_exec["success"], "deterministic tapered L-profile SubCAD code executes")

box_union_l_code = """
import cadquery as cq
from types import SimpleNamespace as Measures
class LBracket:
    def __init__(self, workplane, measures):
        self.model = workplane
        self.measures = measures
        self.build()
    def build(self):
        m = self.measures
        leg_long = cq.Workplane('XY').box(m.leg_long_length, m.leg_width, m.thickness).translate((m.leg_long_length/2, m.leg_width/2, m.thickness/2))
        leg_short = cq.Workplane('XY').box(m.leg_width, m.leg_short_length, m.thickness).translate((m.leg_width/2, m.leg_short_length/2, m.thickness/2))
        base = leg_long.union(leg_short)
        bracket = base.faces('>Z').workplane().center(m.edge_clearance + (m.leg_long_length - 2*m.edge_clearance)/2, m.leg_width/2).rect(m.leg_long_length - 2*m.edge_clearance, m.leg_width - 2*m.edge_clearance, forConstruction=True).vertices().hole(m.hole_diameter)
        bracket = bracket.faces('>Z').workplane().center(m.leg_width/2, m.edge_clearance + (m.leg_short_length - 2*m.edge_clearance)/2).rect(m.leg_width - 2*m.edge_clearance, m.leg_short_length - 2*m.edge_clearance, forConstruction=True).vertices().hole(m.hole_diameter)
        bracket = bracket.edges('|Z').chamfer(m.chamfer_size)
        self.model = bracket
measures = Measures(leg_long_length=80.0, leg_short_length=60.0, leg_width=30.0, thickness=10.0, rib_thickness=5.0, edge_clearance=8.0, hole_diameter=5.0, chamfer_size=0.8)
result = LBracket(cq.Workplane('XY'), measures).model
"""
box_union_l_subcad = build_deterministic_subcad_code(box_union_l_code, [])
check(box_union_l_subcad is not None, "planner builds deterministic box-union L SubCAD code")
check(
    "Stock.rectangular(80, 60, 10)" in box_union_l_subcad
    and box_union_l_subcad.count(".drill(5, through=True") == 6
    and "cx=8, cy=-7" in box_union_l_subcad,
    "deterministic box-union L code preserves construction-vertex hole pattern",
)
box_union_l_exec = run_subcad(box_union_l_subcad)
check(box_union_l_exec["success"], "deterministic box-union L SubCAD code executes")

box_union_profile_code = """
import cadquery as cq
bracket_width=80.0
bracket_height=60.0
plate_thickness=8.0
horizontal=cq.Workplane('XY').box(bracket_width, plate_thickness, plate_thickness)
vertical=cq.Workplane('XY').box(plate_thickness, bracket_height, plate_thickness).translate((0, bracket_height/2+plate_thickness/2,0))
result=horizontal.union(vertical)
"""
box_union_profile_subcad = build_deterministic_subcad_code(box_union_profile_code, [])
check(box_union_profile_subcad is not None, "planner builds deterministic box-union profile SubCAD code")
check(
    "Stock.rectangular(80, 68, 8)" in box_union_profile_subcad
    and ".profile_cutout({'type': 'polygon'" in box_union_profile_subcad,
    "deterministic box-union profile code preserves union outline",
)
box_union_profile_exec = run_subcad(box_union_profile_subcad)
check(box_union_profile_exec["success"], "deterministic box-union profile SubCAD code executes")

top_rib_code = """
plate_length = 80.0
plate_width = 60.0
plate_thickness = 5.0
rib_height = 6.0
rib_width = 20.0
rib_margin = 10.0
mount_hole_diameter = 6.0
mount_hole_spacing_x = 60.0
mount_hole_spacing_y = 40.0
chamfer_size = 1.0
fillet_radius = 0.5
base_sketch = cq.Sketch().rect(plate_length, plate_width)
result = cq.Workplane("XY").placeSketch(base_sketch).extrude(plate_thickness)
rib_sketch = cq.Sketch().rect(rib_width, plate_width - 2 * rib_margin)
rib_center_x = plate_length/2 - rib_margin - rib_width/2
result = (
    result
    .faces(">Z")
    .workplane()
    .center(rib_center_x, 0)
    .placeSketch(rib_sketch)
    .extrude(rib_height)
)
mount_points = [
    (-mount_hole_spacing_x/2, -mount_hole_spacing_y/2),
    ( mount_hole_spacing_x/2, -mount_hole_spacing_y/2),
    ( mount_hole_spacing_x/2,  mount_hole_spacing_y/2),
    (-mount_hole_spacing_x/2,  mount_hole_spacing_y/2),
]
result = result.faces(">Z").workplane().pushPoints(mount_points).hole(mount_hole_diameter)
result = result.edges("|Z").chamfer(chamfer_size)
result = result.edges(">Z").fillet(fillet_radius)
"""
top_rib_subcad = build_deterministic_subcad_code(top_rib_code, [])
check(top_rib_subcad is not None, "planner builds deterministic top-rib SubCAD code")
check(
    "Stock.rectangular(80, 60, 11)" in top_rib_subcad
    and ".rib(width=40, length=20, height=6, cx=20, cy=0)" in top_rib_subcad,
    "deterministic top-rib code preserves stock and rib dimensions",
)
check(".drill(6, through=True, cx=-30, cy=-20)" in top_rib_subcad,
      "deterministic top-rib code includes source pushPoint drills")
top_rib_exec = run_subcad(top_rib_subcad)
check(top_rib_exec["success"], "deterministic top-rib SubCAD code executes")

polar_plan = plan_pure_subcad_features(
    [],
    """
    rib_radius = 36.0
    rib_count = 6
    rib_width = 4.0
    rib_thickness = 1.5
    result = (
        result.faces(">Z")
        .workplane()
        .polarArray(radius=rib_radius, count=rib_count, startAngle=0, angle=360)
        .rect(rib_width, rib_thickness)
        .extrude(2.0)
    )
    """,
)
polar_evidence = [
    feature.evidence for feature in polar_plan.features
    if feature.evidence.get("suggested_subcad")
]
check(
    polar_evidence and len(polar_evidence[0].get("profiles", [])) == 6,
    "planner extracts polarArray retained-rib evidence",
)
polar_rib_code = """
outer_diameter = 80.0
inner_diameter = 30.0
plate_thickness = 8.0
rib_width = 4.0
rib_height = 2.0
rib_thickness = 1.5
rib_count = 6
mount_hole_dia = 5.0
mount_hole_offset = 20.0
base = cq.Workplane("XY").circle(outer_diameter / 2).extrude(plate_thickness)
rib_radius = outer_diameter / 2 - rib_width / 2 - 2.0
result = (
    base.faces(">Z").workplane()
    .polarArray(radius=rib_radius, count=rib_count, startAngle=0, angle=360)
    .rect(rib_width, rib_thickness)
    .extrude(rib_height)
)
result = result.faces("<Z").workplane().hole(inner_diameter)
mount_radius = inner_diameter / 2 + mount_hole_offset
result = (
    result.faces("<Z").workplane()
    .polarArray(radius=mount_radius, count=4, startAngle=0, angle=360)
    .hole(mount_hole_dia)
)
"""
polar_rib_subcad = build_deterministic_subcad_code(polar_rib_code, [])
check(polar_rib_subcad is not None, "planner builds deterministic cylindrical polar-rib SubCAD code")
check(
    "Stock.cylindrical(80, 10)" in polar_rib_subcad
    and ".machine_around_profiles(" in polar_rib_subcad,
    "deterministic polar-rib code preserves cylindrical stock and retained ribs",
)
check(".drill(30, through=True, cx=0.0, cy=0.0)" in polar_rib_subcad,
      "deterministic polar-rib code includes center bore")
check(".drill(5, through=True, cx=35, cy=0)" in polar_rib_subcad,
      "deterministic polar-rib code includes polar mount holes")
polar_rib_exec = run_subcad(polar_rib_subcad)
check(polar_rib_exec["success"], "deterministic cylindrical polar-rib SubCAD code executes")

shell_cover_code = """
outer_radius = 40
cover_height = 10
shell_thickness = 2
sensor_pocket_width = 15
sensor_pocket_length = 20
sensor_pocket_depth = 5
sensor_offset = 20
fillet_radius = 1
hole_diameter = 5
mount_radius = 30
cable_hole_diameter = 3
outer = cq.Workplane('XY').circle(outer_radius).extrude(cover_height)
inner = cq.Workplane('XY').circle(outer_radius - shell_thickness).extrude(cover_height)
cover_shell = outer.cut(inner)
pocketed = cover_shell.faces('>Z').workplane().center(sensor_offset, 0).rect(sensor_pocket_width, sensor_pocket_length).cutBlind(-sensor_pocket_depth)
holes = pocketed.faces('>Z').workplane().polarArray(mount_radius, 0, 360/3, 3).hole(hole_diameter)
cable = holes.faces('>Z').workplane().center(sensor_offset, 0).circle(cable_hole_diameter/2).cutBlind(-sensor_pocket_depth)
result = cable.edges('|Z').fillet(fillet_radius)
"""
shell_cover_subcad = build_deterministic_subcad_code(shell_cover_code, [])
check(shell_cover_subcad is not None, "planner builds deterministic cylindrical shell-cover SubCAD code")
check(
    "Stock.cylindrical(80, 10)" in shell_cover_subcad
    and ".circular_pocket(76, depth=10" in shell_cover_subcad
    and ".edge_fillet(\"|Z\", radius=1)" in shell_cover_subcad,
    "deterministic shell-cover code preserves hollow shell and vertical fillets",
)
shell_cover_exec = run_subcad(shell_cover_subcad)
check(shell_cover_exec["success"], "deterministic cylindrical shell-cover SubCAD code executes")

inner_rib_cage_code = """
cage_length = 80.0
outer_diameter = 70.0
wall_thickness = 5.0
inner_diameter = outer_diameter - 2 * wall_thickness
outer_radius = outer_diameter / 2.0
inner_radius = inner_diameter / 2.0
rib_width = 3.0
rib_height = (outer_radius - inner_radius) - 2.0
rib_spacing_angle = 30.0
fillet_radius = 4.0
outer_cyl = cq.Workplane("XY").circle(outer_radius).extrude(cage_length).edges(">Z").fillet(fillet_radius).edges("<Z").fillet(fillet_radius)
inner_hole = cq.Workplane("XY").circle(inner_radius).extrude(cage_length)
cage_base = outer_cyl.cut(inner_hole)
rib_center_offset = inner_radius + rib_height / 2.0
single_rib = cq.Workplane("XY").rect(rib_width, rib_height).extrude(cage_length).translate((rib_center_offset, 0, 0))
num_ribs = int(360.0 / rib_spacing_angle)
result = cage_base
for i in range(num_ribs):
    angle = i * rib_spacing_angle
    rib = single_rib.rotate((0, 0, 0), (0, 0, 1), angle)
    result = result.union(rib)
"""
inner_rib_cage_subcad = build_deterministic_subcad_code(inner_rib_cage_code, [])
check(inner_rib_cage_subcad is not None, "planner builds deterministic inner-rib cage SubCAD code")
check(
    "Stock.cylindrical(70, 80)" in inner_rib_cage_subcad
    and ".circular_pocket(60, depth=80" in inner_rib_cage_subcad
    and "'type': 'rib'" not in inner_rib_cage_subcad,
    "deterministic inner-rib cage code preserves bore and omits absorbed ribs",
)
inner_rib_cage_exec = run_subcad(inner_rib_cage_subcad)
check(inner_rib_cage_exec["success"], "deterministic cylindrical inner-rib cage SubCAD code executes")

split_shroud_code = """
outer_radius = 45.0
inner_radius = 20.0
shroud_height = 30.0
wall_thickness = 3.0
split_gap = 0.5
chamfer_size = 1.0
bolt_hole_dia = 4.0
num_bolts = 6
bolt_pattern_radius = 35.0
rib_dia = 10.0
rib_height = 10.0
outer = cq.Workplane('XY').circle(outer_radius).extrude(shroud_height)
inner = cq.Workplane('XY').circle(inner_radius).extrude(shroud_height)
base = outer.cut(inner)
split_slab = cq.Workplane('XY').box(split_gap, outer_radius * 4, shroud_height)
base = base.cut(split_slab)
base = base.edges("<X").chamfer(chamfer_size)
bolt_points = [
    (bolt_pattern_radius * math.cos(math.radians(i * 360 / num_bolts)),
     bolt_pattern_radius * math.sin(math.radians(i * 360 / num_bolts)))
    for i in range(num_bolts)
]
base = base.faces(">Z").workplane().pushPoints(bolt_points).hole(bolt_hole_dia)
boss = cq.Workplane('XY').center(inner_radius - wall_thickness / 2, 0).circle(rib_dia / 2).extrude(rib_height).translate((0, 0, shroud_height / 2 - rib_height / 2))
base = base.union(boss)
result = base
"""
split_shroud_subcad = build_deterministic_subcad_code(split_shroud_code, [])
check(split_shroud_subcad is not None, "planner builds deterministic split-shroud SubCAD code")
check(
    "Stock.cylindrical(90, 30)" in split_shroud_subcad
    and ".circular_pocket(40, depth=30" in split_shroud_subcad
    and split_shroud_subcad.count(".drill(4, through=True") == 6,
    "deterministic split-shroud code preserves bore, split, and bolt pattern",
)
split_shroud_exec = run_subcad(split_shroud_subcad)
check(split_shroud_exec["success"], "deterministic split-shroud SubCAD code executes")

annular_rib_code = """
plate_diameter = 80.0
plate_thickness = 5.0
blind_hole_diameter = 20.0
blind_hole_depth = 3.5
rib_offset = 5.0
rib_width = 6.0
rib_height = 3.0
mount_hole_diameter = 5.0
mount_hole_radius = 30.0
chamfer_distance = 0.5
base = cq.Workplane("XY").circle(plate_diameter/2).extrude(plate_thickness)
base = base.faces(">Z").workplane().hole(blind_hole_diameter, blind_hole_depth)
outer_rib_radius = plate_diameter/2 - rib_offset
inner_rib_radius = outer_rib_radius - rib_width
rib = cq.Workplane("XY").circle(outer_rib_radius).extrude(rib_height)
inner_cut = cq.Workplane("XY").circle(inner_rib_radius).extrude(rib_height)
rib = rib.cut(inner_cut)
rib = rib.translate((0, 0, plate_thickness))
result = base.union(rib)
result = result.faces("<Z").workplane().pushPoints([
    (mount_hole_radius, 0),
    (0, mount_hole_radius),
    (-mount_hole_radius, 0),
    (0, -mount_hole_radius)
]).hole(mount_hole_diameter)
result = result.edges(">Z").chamfer(chamfer_distance)
"""
annular_rib_subcad = build_deterministic_subcad_code(annular_rib_code, [])
check(annular_rib_subcad is not None, "planner builds deterministic annular-rib SubCAD code")
check(
    "Stock.cylindrical(80, 8)" in annular_rib_subcad
    and ".machine_around_cylinder(70, height=3" in annular_rib_subcad
    and ".circular_pocket(58, depth=3" in annular_rib_subcad,
    "deterministic annular-rib code preserves ring dimensions",
)
check("base_height=1.5" in annular_rib_subcad,
      "deterministic annular-rib code places blind center recess in the base band")
annular_rib_exec = run_subcad(annular_rib_subcad)
check(annular_rib_exec["success"], "deterministic annular-rib SubCAD code executes")

flange_collar_code = """
flange_radius = 40.0
flange_thickness = 5.0
collar_height = 10.0
collar_outer_radius = 15.0
collar_inner_radius = 10.0
collar_fillet = 0.6
hole_diameter = 2.0
hole_spacing = 10.0
hole_pattern_rows = 6
hole_pattern_cols = 8
hole_clearance_from_collar = 5.0
hole_clearance_from_edge = 5.0
base = cq.Workplane("XY").circle(flange_radius).extrude(flange_thickness)
points = []
col_offset = -(hole_pattern_cols // 2) * hole_spacing
row_offset = -(hole_pattern_rows // 2) * hole_spacing
for i in range(hole_pattern_cols + 1):
    for j in range(hole_pattern_rows + 1):
        x = col_offset + i * hole_spacing
        y = row_offset + j * hole_spacing
        r2 = x * x + y * y
        if r2 < (flange_radius - hole_clearance_from_edge) ** 2 and r2 > (collar_outer_radius + hole_clearance_from_collar) ** 2:
            points.append((x, y))
if points:
    base = base.faces(">Z").workplane().pushPoints(points).hole(hole_diameter)
collar = (
    cq.Workplane("XY", origin=(0, 0, flange_thickness))
    .circle(collar_outer_radius)
    .circle(collar_inner_radius)
    .extrude(collar_height)
)
collar = collar.fillet(collar_fillet)
result = base.union(collar)
"""
flange_collar_subcad = build_deterministic_subcad_code(flange_collar_code, [])
check(flange_collar_subcad is not None, "planner builds deterministic flange-collar SubCAD code")
check(
    "Stock.cylindrical(80, 15)" in flange_collar_subcad
    and ".machine_around_cylinder(30, height=10" in flange_collar_subcad
    and ".circular_pocket(20, depth=10" in flange_collar_subcad,
    "deterministic flange-collar code preserves flange and collar dimensions",
)
check(
    flange_collar_subcad.count(".drill(2, through=True") == 24,
    "deterministic flange-collar code preserves filtered hole grid",
)
flange_collar_exec = run_subcad(flange_collar_subcad)
check(flange_collar_exec["success"], "deterministic flange-collar SubCAD code executes")

grid_boss_code = """
outer_diameter = 80.0
plate_thickness = 8.0
central_hole_diameter = 30.0
blind_hole_depth = 5.0
vent_hole_diameter = 8.0
vent_hole_radius = 30.0
emboss_diameter = 5.0
emboss_height = 2.0
emboss_grid_rows = 5
emboss_grid_cols = 5
emboss_spacing = 10.0
edge_chamfer = 1.0
base = cq.Workplane("XY").circle(outer_diameter / 2).extrude(plate_thickness)
blind = cq.Workplane("XY").circle(central_hole_diameter / 2).extrude(blind_hole_depth)
base = base.cut(blind)
base = (
    base.faces(">Z").workplane()
    .polarArray(count=4, startAngle=0, angle=360, radius=vent_hole_radius)
    .hole(vent_hole_diameter)
)
base = base.edges(">Z").chamfer(edge_chamfer)
emboss = (
    base.faces(">Z").workplane()
    .rarray(emboss_spacing, emboss_spacing, emboss_grid_cols, emboss_grid_rows, center=True)
    .circle(emboss_diameter / 2)
    .extrude(emboss_height)
)
result = base.union(emboss)
"""
grid_boss_subcad = build_deterministic_subcad_code(grid_boss_code, [])
check(grid_boss_subcad is not None, "planner builds deterministic cylindrical grid-boss SubCAD code")
check(
    "Stock.cylindrical(80, 10)" in grid_boss_subcad
    and grid_boss_subcad.count("'type': 'circle'") == 25,
    "deterministic grid-boss code preserves stock and 5x5 boss array",
)
check("machine_around_cylinder(78, height=0.8" in grid_boss_subcad,
      "deterministic grid-boss code adds cylindrical chamfer relief")
grid_boss_exec = run_subcad(grid_boss_subcad)
check(grid_boss_exec["success"], "deterministic cylindrical grid-boss SubCAD code executes")

washer_code = """
outer_diameter = 80.0
inner_diameter = 30.0
thickness = 6.0
mounting_hole_diameter = 5.0
mount_radius = outer_diameter/2 - 8.0
chamfer_dist = 0.7
base = (
    cq.Workplane("XY")
    .circle(outer_diameter/2)
    .circle(inner_diameter/2)
    .extrude(thickness)
)
base = (
    base.faces(">Z").workplane()
    .polarArray(radius=mount_radius, count=4, startAngle=0, angle=360)
    .hole(mounting_hole_diameter)
)
base = base.edges().chamfer(chamfer_dist)
result = base
"""
washer_subcad = build_deterministic_subcad_code(washer_code, [])
check(washer_subcad is not None, "planner builds deterministic cylindrical washer SubCAD code")
check(
    "Stock.cylindrical(80, 6)" in washer_subcad
    and ".drill(30, through=True, cx=0.0, cy=0.0)" in washer_subcad,
    "deterministic washer code preserves outer and inner diameters",
)
check(".edge_chamfer(\"all_edges\", width=0.7)" in washer_subcad,
      "deterministic washer code preserves global chamfer")
washer_exec = run_subcad(washer_subcad)
check(washer_exec["success"], "deterministic cylindrical washer SubCAD code executes")

star_plate_code = """
import math
outer_diameter = 100.0
plate_thickness = 6.0
star_points = 5
star_outer_radius = 35.0
star_inner_radius = 15.0
blind_hole_diameter = 9.0
blind_hole_depth = plate_thickness * 0.8
chamfer_size = 0.8
mount_hole_diameter = 6.0
mount_hole_radius = outer_diameter/2 - 12.0
rib_width = 8.0
rib_depth = plate_thickness * 0.5
rib_spacing_angle = 30.0
result = cq.Workplane("XY").circle(outer_diameter/2).extrude(plate_thickness)
angle_step = math.pi / star_points
pts = []
for i in range(2*star_points):
    r = star_outer_radius if i % 2 == 0 else star_inner_radius
    angle = i * angle_step
    pts.append((r*math.cos(angle), r*math.sin(angle)))
star_cut = cq.Workplane("XY").polyline(pts).close().extrude(-plate_thickness)
result = result.cut(star_cut)
result = result.faces(">Z").workplane().hole(blind_hole_diameter, blind_hole_depth)
mount_pts = []
for i in range(4):
    angle = math.radians(i*90)
    mount_pts.append((mount_hole_radius*math.cos(angle), mount_hole_radius*math.sin(angle)))
result = result.faces(">Z").workplane().pushPoints(mount_pts).hole(mount_hole_diameter)
num_pockets = int(360 / rib_spacing_angle)
for i in range(num_pockets):
    angle = math.radians(i * rib_spacing_angle)
    offset = outer_diameter/2 - rib_width/2 - 5
    x = offset * math.cos(angle)
    y = offset * math.sin(angle)
    result = result.faces(">Z").workplane().center(x, y).rect(rib_width, rib_width).cutBlind(rib_depth)
result = result.edges().chamfer(chamfer_size)
"""
star_plate_subcad = build_deterministic_subcad_code(star_plate_code, [])
check(star_plate_subcad is not None, "planner builds deterministic star-plate SubCAD code")
check(
    "Stock.cylindrical(100, 6)" in star_plate_subcad
    and ".profile_pocket({'type': 'polygon'" not in star_plate_subcad
    and star_plate_subcad.count(".profile_pocket(") == 12,
    "deterministic star-plate code omits no-op star cut and preserves radial pockets",
)
check(".drill(6, through=True, cx=38, cy=0)" in star_plate_subcad,
      "deterministic star-plate code preserves mount holes")
star_plate_exec = run_subcad(star_plate_subcad)
check(star_plate_exec["success"], "deterministic cylindrical star-plate SubCAD code executes")


print("\n2. Fluent pure operation API serializes process-plan records ...")
profile = {"type": "polygon", "points": [(-10, -5), (10, -5), (12, 0), (10, 5), (-10, 5)]}
part = (
    Stock.rectangular(80, 50, 20)
    .edge_chamfer({"face": ">Z", "edge_direction": "X"}, 0.5)
    .edge_fillet({"feature_id": "outer_blend"}, 1.0)
    .profile_pocket(profile, 2.0)
    .profile_cutout(profile, depth=3.0)
    .profile_contour(profile, 4.0)
    .machine_around_profile(profile, 3.0)
    .machine_around_profiles([
        {"type": "rib", "width": 2, "length": 8, "cx": -8, "cy": 0},
        {"type": "rib", "width": 2, "length": 8, "cx": 8, "cy": 0},
    ], 2.0)
    .machine_around_cylinder(12.0, 4.0, cx=5.0, cy=0.0)
    .rib(4.0, 30.0, 5.0, cx=0.0, cy=0.0)
    .pad(profile, 2.0)
    .turn_profile([(0, 0), (10, 0), (10, 20)], stock_diameter=30.0)
    .turn_face(0.0)
    .turn_od(20.0, 40.0)
    .turn_id(10.0, 20.0)
    .turn_groove(2.0, 1.0, -5.0)
    .turn_thread(20.0, 1.5, 15.0)
    .mill_turn_setup()
    .thin_wall_pocket(profile, 1.2, 5.0, open_faces=[">Z"])
    .tube_profile({"diameter": 20}, {"diameter": 10}, 30.0)
)
plan_dict = part.process_plan()
operation_names = [op["operation"] for op in plan_dict["operations"]]
expected = {
    "edge_chamfer",
    "edge_fillet",
    "profile_pocket",
    "profile_cutout",
    "profile_contour",
    "machine_around_profile",
    "machine_around_profiles",
    "machine_around_cylinder",
    "rib",
    "pad",
    "turn_profile",
    "turn_face",
    "turn_od",
    "turn_id",
    "turn_groove",
    "turn_thread",
    "mill_turn_setup",
    "thin_wall_pocket",
    "tube_profile",
}
check(expected.issubset(set(operation_names)), "all pure coverage operations appear in process plan")
for op in plan_dict["operations"]:
    if op["operation"] in expected:
        check(op.get("manufacturing_completeness") == ("requires_validation" if op.get("process") == "turn"
              or op["operation"] in {"mill_turn_setup", "thin_wall_pocket"} else "pure_operation"),
              f"{op['operation']} marks pure operation completeness")
        if op["operation"] != "mill_turn_setup":
            check("toolpath" in op, f"{op['operation']} emits neutral toolpath")

# Public operation names do not prove geometry coverage. Unimplemented
# operations must reject execution instead of preserving an unchanged shape.
for name, arguments in (
    ("surface_mill", {"surface_ref": {"surface": "freeform_patch"}}),
    ("sweep_mill", {"profile": profile, "path": {"points": [(0, 0), (20, 0)]}}),
    ("loft_mill", {"profiles": [profile, {"type": "circle", "diameter": 8}]}),
    ("hollow_bore", {"inner_profile": {"diameter": 10}, "outer_profile": {"diameter": 20}}),
    ("wire_cut_profile", {"profile": profile, "thickness": 6}),
    ("wire_cut_internal", {"profile": profile, "thickness": 6}),
):
    try:
        create_operation(name, **arguments).apply(Stock.rectangular(30, 30, 20)._shape)
    except ValueError as error:
        check(bool(str(error)), f"{name} rejects unavailable native geometry")
    else:
        raise AssertionError(f"{name} must reject an unchanged geometry result")

bottom_face_part = (
    Stock.rectangular(40, 30, 10)
    .pocket(8, 12, 2, face_selector="<Z")
    .circular_pocket(6, 1, face_selector="<Z")
    .drill(3, through=True, face_selector="<Z")
    .slot(10, 3, 1, face_selector="<Z")
    .machine_around_cylinder(5, 2, face_selector="<Z")
)
bottom_ops = bottom_face_part.process_plan()["operations"]
check(all(op.get("face_selector") == "<Z" for op in bottom_ops),
      "fluent core operations preserve bottom face_selector in process plan")


print("\n3. Operation factory exposes new families ...")
factory_op = create_operation("surface_mill", surface_ref={"id": "patch_1"})
check(factory_op.to_dict()["operation"] == "surface_mill",
      "create_operation can build surface_mill")

print("\n4. Geometry-backed pure ops remove material ...")
base = Stock.rectangular(80, 50, 20)
base_volume = base.volume
profile_part = base.profile_pocket(profile, 3.0)
check(profile_part.volume < base_volume, "profile_pocket changes B-Rep volume")
through_pocket = base.profile_pocket(profile, 20.0, through=True)
check(through_pocket.volume < profile_part.volume,
      "profile_pocket supports internal through cuts")
cutout_part = base.profile_cutout(profile, depth=4.0)
check(cutout_part.volume < base_volume, "profile_cutout changes B-Rep volume")
through_cutout = base.profile_cutout(profile, through=True)
check(through_cutout.volume < base_volume, "through profile_cutout retains profile outline")
after_cutout_pocket = through_cutout.pocket(width=2.0, length=4.0, depth=1.0, cx=0.0, cy=0.0)
check(after_cutout_pocket.volume < through_cutout.volume,
      "profile_cutout keeps a usable top face for later operations")
hex_cutout = Stock.rectangular(100, 90, 12).profile_cutout(
    {"type": "polygon", "n_sides": 6, "circumradius": 30.0},
    through=True,
)
check(hex_cutout.volume > 20000, "profile_cutout supports regular polygon profile dicts")
hex_depth_cutout = Stock.rectangular(100, 90, 12).profile_cutout(
    {"type": "polygon", "n_sides": 6, "circumradius": 30.0},
    depth=12.0,
)
check(hex_depth_cutout.volume > 20000,
      "profile_cutout treats full-depth outline cuts as through cutouts")
hex_diameter_cutout = Stock.rectangular(100, 90, 12).profile_cutout(
    {"type": "polygon", "n_sides": 6, "circumdiameter": 60.0},
    through=True,
)
check(abs(hex_diameter_cutout.volume - hex_cutout.volume) < 1e-6,
      "regular polygon profiles support circumdiameter")
contour_part = base.profile_contour(profile, 3.0)
check(contour_part.volume < base_volume, "profile_contour changes B-Rep volume")
around_profile = base.machine_around_profile(profile, 3.0)
check(around_profile.volume < base_volume, "machine_around_profile removes surrounding material")
around_cylinder = base.machine_around_cylinder(10.0, 3.0)
check(around_cylinder.volume < base_volume, "machine_around_cylinder removes surrounding material")
selected_chamfer = base.edge_chamfer({"face": ">Z", "edge_direction": "X"}, 0.5)
check(selected_chamfer.volume < base_volume, "edge_chamfer changes selected-edge geometry")
selected_fillet = base.edge_fillet({"face": ">Z", "edge_direction": "X"}, 0.5)
check(selected_fillet.volume <= base_volume, "edge_fillet executes without adding material")

taper_delta = 10.0 * math.tan(math.radians(5.0))
tapered = (
    Stock.cylindrical(2 * (40.0 + taper_delta), 10.0)
    .turn_profile(
        {
            "type": "tapered_cylinder",
            "bottom_diameter": 80.0,
            "top_diameter": 2 * (40.0 + taper_delta),
            "height": 10.0,
        },
        stock_diameter=2 * (40.0 + taper_delta),
    )
)
check(tapered.volume < Stock.cylindrical(2 * (40.0 + taper_delta), 10.0).volume,
      "turn_profile tapered_cylinder changes B-Rep volume")

print("\nALL PASSED")
