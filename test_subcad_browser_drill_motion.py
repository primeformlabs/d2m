"""Verify drilling frames and physical point travel independently of browser grids."""
import math
import numpy as np
import pytest

cq = pytest.importorskip("cadquery")
from src.subcad import Stock, ToolSpec
from src.subcad.operations import DrillOp, SpotDrillOp

FACES = [">X", "<X", ">Y", "<Y", ">Z", "<Z"]


def tool():
    return ToolSpec("drill", 4, tip_angle=118, flute_length=40, catalog_id="ENGINEERING-DRILL",
        physical_assembly={"owned": True, "assembly_id": "ENGINEERING-DRILL"})


@pytest.mark.parametrize("face", FACES)
@pytest.mark.parametrize("peck", [False, True])
def test_through_drilling_passes_the_far_face_with_the_full_diameter(face, peck):
    before = Stock.rectangular(24, 20, 16)
    after = before.drill(4, through=True, face_selector=face, cx=1, cy=-1,
        spot_drill=False, peck=peck, tool=tool())
    record = after.process_plan()["operations"][-1]
    path = record["toolpath"]
    metadata = path["metadata"]
    assert metadata["coordinate_system"] == "stock_mm"
    assert metadata["tool_origin"] == metadata["depth_reference"] == "cutting_tip"
    assert metadata["cad_drill_bottom"] == "through"
    assert metadata["verification_status"] == "not_verified"
    axes = np.array([metadata["tool_frame"][key] for key in ("x_axis", "y_axis", "z_axis")])
    np.testing.assert_allclose(axes @ axes.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(axes) == pytest.approx(1)
    normal = axes[2]
    axis = "XYZ".index(face[1])
    bound = before.bounding_box
    surface = bound[f"{'xyz'[axis]}_{'max' if face[0] == '>' else 'min'}"]
    low = min(np.array(move["position"]) @ normal for move in path["moves"])
    far = bound[f"{'xyz'[axis]}_{'min' if face[0] == '>' else 'max'}"] * normal[axis]
    height = 2 / math.tan(math.radians(59))
    assert low == pytest.approx(far - height - 1)
    assert metadata["drill_tip_height_mm"] == pytest.approx(height)
    assert path["moves"][0]["position"][axis] == pytest.approx(surface + normal[axis] * path["safe_z"])
    # Independent material membership for a full-diameter through cylinder.
    points = np.random.default_rng(482).uniform([-12, -10, -8], [12, 10, 8], (1200, 3))
    u, v = {0: (1, 2), 1: (0, 2), 2: (0, 1)}[axis]
    removed = (points[:, u] - 1) ** 2 + (points[:, v] + 1) ** 2 < 4
    inside = np.array([after._shape.val().isInside(tuple(point), 1e-8) for point in points])
    assert np.array_equal(~removed, inside)


@pytest.mark.parametrize("face", FACES)
def test_blind_drilling_states_the_tip_depth_and_retains_the_cad_bottom_limit(face):
    after = Stock.rectangular(24, 20, 16).drill(4, depth=5, face_selector=face, spot_drill=False, tool=tool())
    path = after.process_plan()["operations"][-1]["toolpath"]
    metadata = path["metadata"]
    assert metadata["cad_drill_bottom"] == "flat_intent"
    normal = metadata["tool_frame"]["z_axis"]
    axis = "XYZ".index(face[1])
    surface = metadata["incoming_bounds_mm"][f"{'xyz'[axis]}_{'max' if face[0] == '>' else 'min'}"]
    assert min(move["position"][axis] * normal[axis] for move in path["moves"]) == pytest.approx(surface * normal[axis] - 5)


@pytest.mark.parametrize("face", FACES)
def test_spot_drilling_keeps_the_same_face_and_records_its_planning_only_cad_state(face):
    after = Stock.rectangular(24, 20, 16).drill(4, depth=5, face_selector=face, tool=tool())
    paths = [record["toolpath"] for record in after.process_plan()["operations"]]
    assert len(paths) == 2
    assert paths[0]["metadata"]["face_selector"] == paths[1]["metadata"]["face_selector"] == face
    assert paths[0]["metadata"]["cad_drill_bottom"] == "planning_only"


@pytest.mark.parametrize("operation", [DrillOp, SpotDrillOp])
def test_unapplied_drill_operations_cannot_claim_stock_coordinates(operation):
    with pytest.raises(ValueError, match="incoming stock"):
        operation(tool=tool()).to_toolpath()


@pytest.mark.parametrize("changes", [{"tip_angle": 0}, {"tip_angle": 180}, {"tip_angle": math.nan},
    {"tip_diameter": .2}, {"flute_length": 5}, {"diameter": 3}, {"tip_angle": True}])
def test_missing_point_geometry_and_insufficient_cutting_length_prevent_motion(changes):
    selected = tool()
    for key, value in changes.items():
        setattr(selected, key, value)
    record = Stock.rectangular(24, 20, 16).drill(4, through=True, spot_drill=False, tool=selected).process_plan()["operations"][-1]
    assert record["toolpath_status"] == "unavailable"
