"""Keep recorded native drilling unchanged when exporting browser frames."""
from copy import deepcopy

import numpy as np
import pytest

pytest.importorskip("cadquery")
from src.subcad import Stock, ToolSpec
from src.subcad.operations import DrillOp, SpotDrillOp

FACES = [">X", "<X", ">Y", "<Y", ">Z", "<Z"]


def tool():
    return ToolSpec("drill", 4, tip_angle=118, flute_length=40, catalog_id="ENGINEERING-DRILL",
        physical_assembly={"owned": True, "assembly_id": "ENGINEERING-DRILL"})


@pytest.mark.parametrize("face", FACES)
@pytest.mark.parametrize("peck", [False, True])
def test_browser_binding_keeps_actual_native_drill_moves_and_depth_references(face, peck):
    before = Stock.rectangular(24, 20, 16)
    operation = DrillOp(diameter=4, depth=5, face_selector=face, cx=1, cy=-1,
        spot_drill=False, peck=peck, tool=tool())
    native_result = operation.apply(before._shape)
    native_path = deepcopy(operation.to_toolpath().to_dict())
    result = before._apply_op(operation)
    path = result.process_plan()["operations"][-1]["toolpath"]
    metadata = path["metadata"]
    assert path["moves"] == native_path["moves"]
    assert path["safe_z"] == native_path["safe_z"]
    assert all(metadata[key] == value for key, value in native_path["metadata"].items())
    assert result.volume == pytest.approx(native_result.val().Volume(), abs=1e-6)
    assert metadata["coordinate_system"] == "part_mm"
    assert metadata["depth_reference"] == "full_diameter"
    assert metadata["tool_origin"] == "cutting_tip"
    assert metadata["incoming_bounds_mm"] == before.bounding_box
    axes = np.array([metadata["tool_frame"][key] for key in ("x_axis", "y_axis", "z_axis")])
    np.testing.assert_allclose(axes @ axes.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(axes) == pytest.approx(1)
    np.testing.assert_allclose(axes[2], native_path["metadata"]["tool_axis"], atol=1e-12)
    assert metadata["verification_status"] == "not_verified"


@pytest.mark.parametrize("face", FACES)
def test_spot_drill_keeps_native_tip_depth_and_actual_material(face):
    before = Stock.rectangular(24, 20, 16)
    operation = SpotDrillOp(hole_diameter=4, depth=.5, face_selector=face, tool=tool())
    native_result = operation.apply(before._shape)
    native_path = deepcopy(operation.to_toolpath().to_dict())
    result = before._apply_op(operation)
    path = result.process_plan()["operations"][-1]["toolpath"]
    assert path["moves"] == native_path["moves"]
    assert result.volume == pytest.approx(native_result.val().Volume(), abs=1e-6)
    assert result.volume < before.volume
    assert path["metadata"]["depth_reference"] == "tip"
    assert path["metadata"]["tool_origin"] == "cutting_tip"
    assert path["metadata"]["incoming_bounds_mm"] == before.bounding_box


@pytest.mark.parametrize("operation", [DrillOp, SpotDrillOp])
def test_unapplied_drill_plans_have_no_browser_stock_binding(operation):
    path = operation(tool=tool()).to_toolpath()
    assert path.metadata.get("tool_origin") != "cutting_tip"
    assert "incoming_bounds_mm" not in path.metadata
