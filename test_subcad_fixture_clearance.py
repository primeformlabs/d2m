"""Focused regression checks for fixture travel, cutter axes, and setup identity."""

from copy import deepcopy
from unittest import TestCase, main
from unittest.mock import patch

from src.subcad.fixture_sweep import FACE_AXES
from src.subcad.fixturing import ClampingZone, FixtureSpec, Setup, check_operation_against_fixture
from src.subcad.process_plan import ProcessPlan
from src.subcad.stock import Stock
from src.subcad.validation import _check_fixture_clearance


def tool():
    return {"catalog_id": "owned_tool", "assembly": {
        "cutter_diameter_mm": 0.2, "flute_length_mm": 4,
        "shank_diameter_mm": 0.2, "stickout_mm": 8,
        "holder_diameter_mm": 2, "holder_length_mm": 4,
    }}


def zone(label="clamp", **changes):
    return {"label": label, "x_min": 0.9, "x_max": 1.1,
            "y_min": -0.1, "y_max": 0.1, "z_min": 1, "z_max": 2,
            "margin_mm": 0, **changes}


def operation(face=">Z", points=None):
    return {"operation": "pocket", "setup": "first", "face_selector": face, "tool": tool(),
            "toolpath": {"metadata": {"coordinate_system": "part_mm", "tool_axis": list(FACE_AXES[face])},
                         "moves": [{"type": "feed", "position": point}
                                   for point in (points or [[0, 0, 0], [2, 0, 0]])]}}


def plan(op=None, zones=None):
    return {"fixture": {"name": "first_fixture", "clamping_zones": zones or [zone()]},
            "setups": {"first": {"fixture": "first_fixture", "face_selector": ">Z"}},
            "operations": [op or operation()]}


class FixtureClearanceTests(TestCase):
    def test_thin_clamp_between_old_samples(self):
        hits = _check_fixture_clearance(plan())
        cutter = next(hit for hit in hits if hit.context.get("component") == "cutter")
        self.assertAlmostEqual(cutter.context["point"][0], 0.9)
        self.assertEqual(cutter.context["method"], "continuous_cardinal_envelope")
        self.assertFalse(cutter.context["manufacturing_ready"])

    def test_all_six_axes_and_three_components(self):
        axial_ranges = {"cutter": (1, 2), "shank": (5, 6), "holder": (9, 10)}
        for face, axis in FACE_AXES.items():
            axial = next(index for index, value in enumerate(axis) if value)
            radial = next(index for index in range(3) if index != axial)
            start, end = [0.0] * 3, [0.0] * 3
            end[radial] = 2
            for component, (low, high) in axial_ranges.items():
                with self.subTest(face=face, component=component):
                    bounds = zone()
                    for index, key in enumerate(("x", "y", "z")):
                        limits = sorted((low * axis[index], high * axis[index])) if index == axial else (0.9, 1.1) if index == radial else (-0.1, 0.1)
                        bounds[key + "_min"], bounds[key + "_max"] = limits
                    issues = _check_fixture_clearance(plan(operation(face, [start, end]), [bounds]))
                    hit = next(hit for hit in issues if hit.context.get("component") == component)
                    self.assertEqual(hit.context["tool_axis"], list(axis))
                    self.assertEqual(hit.context["verification_status"], "possible_collision")

    def test_diagonal_distance_avoids_swept_box_false_positive(self):
        bounds = zone(x_min=-0.1, x_max=0.1, y_min=9.9, y_max=10.1)
        issues = _check_fixture_clearance(plan(operation(points=[[0, 0, 0], [10, 10, 0]]), [bounds]))
        self.assertFalse([hit for hit in issues if hit.context.get("component")])

    def test_axial_and_radial_overlap_must_happen_at_same_time(self):
        bounds = zone(x_min=9, x_max=10, z_min=0, z_max=1)
        issues = _check_fixture_clearance(plan(operation(points=[[0, 0, 0], [10, 0, 30]]), [bounds]))
        self.assertFalse([hit for hit in issues if hit.context.get("component")])

    def test_axial_margin_is_applied(self):
        bounds = zone(x_min=-1, x_max=1, z_min=-0.5, z_max=-0.1, margin_mm=0.2)
        issues = _check_fixture_clearance(plan(operation(points=[[0, 0, 0]]), [bounds]))
        self.assertTrue([hit for hit in issues if hit.context.get("component") == "cutter"])

    def test_entry_position_is_checked(self):
        op = operation(points=[[2, 0, 0]])
        op["toolpath"]["metadata"]["pass_entry_position"] = [0, 0, 0]
        issues = _check_fixture_clearance(plan(op))
        self.assertTrue([hit for hit in issues if hit.context.get("component") == "cutter"])

    def test_fixture_snapshot_wins_over_latest_fixture(self):
        value = plan()
        value["setups"]["first"]["fixture_spec"] = deepcopy(value["fixture"])
        value["fixture"] = {"name": "latest_fixture", "clamping_zones": [zone(x_min=100, x_max=101)]}
        hits = _check_fixture_clearance(value)
        self.assertEqual(next(hit.context["fixture_id"] for hit in hits if hit.context.get("component")), "first_fixture")

    def test_missing_snapshot_does_not_use_latest_fixture(self):
        value = plan()
        value["fixture"]["name"] = "latest_fixture"
        issues = _check_fixture_clearance(value)
        self.assertTrue(any("latest fixture cannot replace" in hit.message for hit in issues))
        self.assertFalse([hit for hit in issues if hit.context.get("component")])

    def test_empty_setup_zones_override_global_zones(self):
        value = plan()
        value["setups"]["first"]["clamping_zones"] = []
        issues = _check_fixture_clearance(value)
        self.assertTrue(any("no collision zones" in hit.message for hit in issues))
        self.assertFalse([hit for hit in issues if hit.context.get("component")])

    def test_unsupported_arc_is_not_replaced_by_chord(self):
        op = operation(points=[[0, 0, 0], [2, 0, 0]])
        op["toolpath"]["moves"][1]["type"] = "arc_cw"
        issues = _check_fixture_clearance(plan(op))
        self.assertTrue(any(hit.context.get("move_type") == "arc_cw" for hit in issues))
        self.assertFalse([hit for hit in issues if hit.context.get("component") == "cutter"])

    def test_invalid_axes_and_coordinates_are_unverified(self):
        for axis in ([1, 1, 0], [0, 0, -1], [float("nan"), 0, 1], [1, 0]):
            with self.subTest(axis=axis):
                op = operation()
                op["toolpath"]["metadata"]["tool_axis"] = axis
                issues = _check_fixture_clearance(plan(op))
                self.assertTrue(any(hit.code == "fixture_clearance_unverified" for hit in issues))
                self.assertFalse([hit for hit in issues if hit.context.get("component")])
        op = operation()
        op["toolpath"]["metadata"]["coordinate_system"] = "machine_mm"
        self.assertTrue(any("part_mm" in hit.message for hit in _check_fixture_clearance(plan(op))))

    def test_invalid_zone_or_path_is_reported(self):
        for changes in ({"x_min": 3, "x_max": 1}, {"z_max": float("inf")}, {"margin_mm": -1}):
            with self.subTest(changes=changes):
                issues = _check_fixture_clearance(plan(zones=[zone(**changes)]))
                self.assertTrue(any(hit.code == "fixture_clearance_unverified" for hit in issues))
        for points in ([[float("nan"), 0, 0]], [[0, 0]], [[float("inf"), 0, 0]]):
            with self.subTest(points=points):
                issues = _check_fixture_clearance(plan(operation(points=points)))
                self.assertTrue(any("position is invalid" in hit.message for hit in issues))

    def test_missing_holder_does_not_claim_complete_clearance(self):
        op = operation(points=[[10, 0, 30], [20, 0, 30]])
        del op["tool"]["assembly"]["holder_diameter_mm"]
        issues = _check_fixture_clearance(plan(op))
        self.assertTrue(any("holder_diameter_mm" in hit.context.get("missing_dimensions", []) for hit in issues))

    def test_stock_checks_each_recorded_setup(self):
        first_fixture = FixtureSpec("first_fixture", clamping_zones=[ClampingZone("first_clamp", 0.9, 1.1, -0.1, 0.1, 1, 2, 0)])
        latest_fixture = FixtureSpec("latest_fixture", clamping_zones=[ClampingZone("latest_clamp", 100, 101, -1, 1, 1, 2, 0)])
        stock = Stock._from_parts(None, "aluminum_6061", {}, ProcessPlan(operations=[operation()]), 2,
                                  fixture=latest_fixture,
                                  setups={"first": Setup("first", first_fixture), "latest": Setup("latest", latest_fixture)},
                                  active_setup="latest")
        self.assertTrue(any("first_clamp" in message for message in stock.validate_fixture_clearance()))
        with self.assertRaises(ValueError):
            stock.new_setup("first")

    def test_setup_snapshot_does_not_change_when_input_fixture_changes(self):
        fixture = FixtureSpec("fixture", clamping_zones=[ClampingZone("clamp", 0, 1, 0, 1, 0, 1)])
        stock = Stock._from_parts(None, "aluminum_6061", {}, ProcessPlan(), 1, fixture=fixture)
        stock = stock.new_setup("first")
        fixture.clamping_zones[0].x_max = 500
        self.assertEqual(stock._setups["first"].to_dict()["fixture_spec"]["clamping_zones"][0]["x_max"], 1)

    def test_validation_failure_is_not_silently_ignored(self):
        with patch("src.subcad.validation.validate_all", side_effect=RuntimeError("fixture check failed")):
            buckets = ProcessPlan().validation_buckets
        self.assertTrue(any("fixture check failed" in message for message in buckets["errors"]))

    def test_fixture_error_severity_survives_plan_buckets(self):
        value = plan(zones=[zone(severity="error")])
        process = ProcessPlan(material="aluminum_6061", fixture=value["fixture"], setups=value["setups"], operations=value["operations"])
        self.assertTrue(any("cutter envelope" in message for message in process.validation_buckets["errors"]))

    def test_unknown_fixture_frame_does_not_invent_collision(self):
        value = plan()
        value["fixture"]["coordinate_frame"] = {"origin": "stock_center_top"}
        issues = _check_fixture_clearance(value)
        self.assertTrue(any("coordinate transform is unverified" in hit.message for hit in issues))
        self.assertFalse([hit for hit in issues if hit.context.get("component")])

    def test_public_single_operation_check_uses_continuous_travel(self):
        fixture = FixtureSpec("first_fixture", clamping_zones=[ClampingZone("thin_clamp", 0.9, 1.1, -0.1, 0.1, 1, 2, 0)])
        self.assertTrue(any("cutter envelope" in message for message in
                            check_operation_against_fixture(operation(), Setup("first", fixture), {})))

    def test_stock_fit_uses_each_setup_fixture(self):
        value = plan()
        value["stock_dimensions"] = {"length": 50, "width": 10}
        value["setups"]["first"]["fixture_spec"] = {**value["fixture"], "max_opening_mm": 20}
        value["fixture"] = {"name": "latest", "max_opening_mm": 100}
        fit = next(hit for hit in _check_fixture_clearance(value) if hit.code == "fixture_stock_fit")
        self.assertEqual(fit.context["fixture_id"], "first_fixture")

    def test_nonfinite_tool_dimensions_are_reported(self):
        value = plan()
        value["operations"][0]["tool"]["assembly"]["holder_diameter_mm"] = float("nan")
        issues = _check_fixture_clearance(value)
        self.assertTrue(any("holder_diameter_mm" in hit.context.get("missing_dimensions", []) for hit in issues))

    def test_native_operation_retains_fixture_setup(self):
        fixture = FixtureSpec("owned_fixture", clamping_zones=[ClampingZone("jaw", 15, 16, -1, 1, 0, 1)])
        stock = Stock.rectangular(20, 20, 10).with_fixture(fixture).new_setup("first")
        stock = stock.drill(diameter=2, cx=0, cy=0, depth=1)
        native = stock._plan.operations[-1]
        self.assertEqual(native["fixture_setup"]["fixture_spec"]["name"], "owned_fixture")
        self.assertEqual(native["fixture_setup"]["name"], native["setup"])
        fixture.clamping_zones[0].x_max = 500
        self.assertEqual(native["fixture_setup"]["fixture_spec"]["clamping_zones"][0]["x_max"], 16)


if __name__ == "__main__":
    main()
