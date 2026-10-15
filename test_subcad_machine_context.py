"""Tests for machine-aware SubCAD validation."""
import sys

from src.subcad import Stock, load_machine_context
from src.subcad.geometry import _HAS_CADQUERY
from src.subcad.validation import validate_all

if not _HAS_CADQUERY:
    print("SKIP: CadQuery not available")
    sys.exit(0)

passed = 0
failed = 0


def check(label, condition, detail=""):
    global passed, failed
    if condition:
        passed += 1
        print(f"  PASS  {label}")
    else:
        failed += 1
        print(f"  FAIL  {label}  -- {detail}")


def codes(report):
    return {issue.code for issue in report.issues}


generic = load_machine_context("machines/generic_3axis.json").to_dict()
dmu50 = load_machine_context("machines/dmg_dmu50.json").to_dict()

print("=" * 60)
print("SubCAD Machine Context Test")
print("=" * 60)

print("\n1. Machine metadata is carried by Stock ...")
part = (
    Stock.rectangular(120, 80, 6)
    .with_machine("machines/generic_3axis.json")
    .drill_at(6, through=True, from_left=10, from_top=12, spot_drill=False)
)
plan = part.process_plan()
check("plan has machine metadata", plan["machine"]["machine_id"] == "generic_3axis", plan.get("machine"))
check("machine axes include XYZ", set(plan["machine"]["axes"]) >= {"X", "Y", "Z"}, plan["machine"]["axes"])
check(
    "generic capability schema is explicit",
    plan["machine"]["capabilities"]["schema_version"] == "subcad.machine_capabilities.v1",
    plan["machine"]["capabilities"],
)
check(
    "generic does not list surface_mill",
    "surface_mill" not in plan["machine"]["capabilities"]["operation_families"],
    plan["machine"]["capabilities"]["operation_families"],
)

print("\n2. 5-axis operation on 3-axis machine is rejected ...")
plan_5axis_on_3axis = {
    "schema_version": "subcad.shop_floor.v1",
    "material": "aluminum_6061",
    "stock_dimensions": {"length": 100, "width": 80, "height": 20},
    "machine": generic,
    "operations": [{"operation": "surface_mill", "process": "5axis", "depth_mm": 1.0}],
}
report = validate_all(plan_5axis_on_3axis, {}, structured=True)
check("3-axis rejects 5-axis operation", "machine_requires_5axis" in codes(report), codes(report))

print("\n3. Same 5-axis operation is allowed on 5-axis machine ...")
plan_5axis_on_dmu = dict(plan_5axis_on_3axis, machine=dmu50)
report = validate_all(plan_5axis_on_dmu, {}, structured=True)
check("5-axis machine accepts 5-axis operation", "machine_requires_5axis" not in codes(report), codes(report))

print("\n3b. Operation family support is checked explicitly ...")
family_plan = {
    "schema_version": "subcad.shop_floor.v1",
    "material": "aluminum_6061",
    "stock_dimensions": {"length": 100, "width": 80, "height": 20},
    "machine": generic,
    "operations": [{"operation": "surface_mill", "process": "mill", "depth_mm": 1.0}],
}
report = validate_all(family_plan, {}, structured=True)
check("unsupported operation family fails", "machine_operation_unsupported" in codes(report), codes(report))

print("\n4. Machine travel limits are checked ...")
oversize_plan = {
    "schema_version": "subcad.shop_floor.v1",
    "material": "aluminum_6061",
    "stock_dimensions": {"length": 700, "width": 80, "height": 20},
    "machine": generic,
    "operations": [{"operation": "drill", "position": [0, 0], "depth_mm": 5}],
}
report = validate_all(oversize_plan, {}, structured=True)
check("oversize stock fails X travel", "machine_travel_x" in codes(report), codes(report))

print("\n5. Side-face work on 3-axis needs setup metadata ...")
side_no_setup = {
    "schema_version": "subcad.shop_floor.v1",
    "material": "aluminum_6061",
    "stock_dimensions": {"length": 100, "width": 80, "height": 20},
    "machine": generic,
    "operations": [{"operation": "drill", "face_selector": ">X", "position": [0, 0], "depth_mm": 5}],
}
report = validate_all(side_no_setup, {}, structured=True)
check("side face without setup is rejected", "machine_setup_required" in codes(report), codes(report))

side_with_setup = {
    **side_no_setup,
    "setups": {"side": {"name": "side", "face_selector": ">X", "work_offset": "G55"}},
    "operations": [{"operation": "drill", "setup": "side", "face_selector": ">X", "position": [0, 0], "depth_mm": 5}],
}
report = validate_all(side_with_setup, {}, structured=True)
check("side face with indexed setup passes machine setup check", "machine_setup_required" not in codes(report), codes(report))

print("\n6. Spindle and feed limits are checked ...")
fast_plan = {
    "schema_version": "subcad.shop_floor.v1",
    "material": "aluminum_6061",
    "stock_dimensions": {"length": 100, "width": 80, "height": 20},
    "machine": generic,
    "operations": [{
        "operation": "slot",
        "position": [0, 0],
        "depth_mm": 5,
        "feeds_speeds": {"spindle_rpm": 12000, "feed_rate_mm_per_min": 50000},
    }],
}
report = validate_all(fast_plan, {}, structured=True)
code_set = codes(report)
check("spindle rpm limit fails", "machine_spindle_rpm_exceeded" in code_set, code_set)
check("feed limit fails", "machine_feed_exceeded" in code_set, code_set)

print("\n7. Controller, coolant, probing, and tool interface are checked ...")
capability_plan = {
    "schema_version": "subcad.shop_floor.v1",
    "material": "aluminum_6061",
    "stock_dimensions": {"length": 100, "width": 80, "height": 20},
    "machine": generic,
    "operations": [
        {"operation": "drill", "position": [0, 0], "controller_cycle": "G234"},
        {"operation": "drill", "position": [5, 0], "coolant": "through_spindle"},
        {"operation": "drill", "position": [10, 0], "requires_probe": True},
        {"operation": "drill", "position": [15, 0], "tool": {"interface": "HSK-A63"}},
    ],
}
report = validate_all(capability_plan, {}, structured=True)
code_set = codes(report)
check("unsupported controller cycle fails", "machine_controller_cycle_unsupported" in code_set, code_set)
check("unsupported coolant mode fails", "machine_coolant_unsupported" in code_set, code_set)
check("unsupported probing fails", "machine_probe_unsupported" in code_set, code_set)
check("tool interface mismatch fails", "machine_tool_interface_mismatch" in code_set, code_set)

print("\n8. Tool capacity is checked ...")
capacity_plan = {
    "schema_version": "subcad.shop_floor.v1",
    "material": "aluminum_6061",
    "stock_dimensions": {"length": 100, "width": 80, "height": 20},
    "machine": generic,
    "tools_used": [
        {"tool_type": "flat_endmill", "diameter_mm": index + 1}
        for index in range(21)
    ],
    "operations": [{"operation": "drill", "position": [0, 0], "depth_mm": 5}],
}
report = validate_all(capacity_plan, {}, structured=True)
check("tool capacity fails", "machine_tool_capacity_exceeded" in codes(report), codes(report))

print("\n9. Strict fluent validation raises errors ...")
try:
    (
        Stock.rectangular(100, 80, 20)
        .with_machine("machines/generic_3axis.json")
        .surface_mill({"type": "freeform"}, tolerance_mm=0.1)
        .validate_shop_floor(raise_on_error=True)
    )
    raised = False
except ValueError as exc:
    raised = "explicit geometry" in str(exc)
check("unresolved surface geometry stops the fluent operation", raised)

print("\n" + "=" * 60)
print(f"Passed: {passed}")
print(f"Failed: {failed}")
print("=" * 60)

if failed:
    sys.exit(1)
