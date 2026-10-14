"""Compare native face paths with independent cutter removal and CAD material."""

import math
import numpy as np
import pytest

cq = pytest.importorskip("cadquery")
from src.subcad import FixtureCatalog, ProcessPlan, Stock, ToolSpec
from src.subcad.operations import FaceMillOp


FACES = [">X", "<X", ">Y", "<Y", ">Z", "<Z"]


def path_of(result):
    record = result.process_plan()["operations"][-1]
    assert "toolpath" in record, record.get("toolpath_reason")
    path = record["toolpath"]
    assert path["metadata"]["coordinate_system"] in {"stock_mm", "part_mm"}
    frame = path["metadata"]["tool_frame"]
    axes = np.array([frame[key] for key in ("x_axis", "y_axis", "z_axis")])
    np.testing.assert_allclose(axes @ axes.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(axes) == pytest.approx(1)
    return path, np.array(frame["z_axis"], dtype=float)


def cutting_segments(path):
    return [(np.array(a["position"]), np.array(b["position"]))
            for a, b in zip(path["moves"], path["moves"][1:])
            if b["type"] in {"feed", "plunge"}]


def cutter_contains(points, path, normal, radius, flute):
    """Independent capsule/prism membership for horizontal or axial moves."""
    removed = np.zeros(len(points), dtype=bool)
    for a, b in cutting_segments(path):
        delta = b - a
        normal_delta = float(delta @ normal)
        lateral = delta - normal_delta * normal
        relative = points - a
        axial = relative @ normal
        projected = relative - axial[:, None] * normal
        length2 = float(lateral @ lateral)
        if length2 > 1e-12:
            assert abs(normal_delta) < 1e-9
            t = np.clip(projected @ lateral / length2, 0, 1)
            projected -= t[:, None] * lateral
        distance2 = np.einsum("ij,ij->i", projected, projected)
        removed |= ((distance2 < radius ** 2) &
                    (axial > min(0, normal_delta)) &
                    (axial < max(0, normal_delta) + flute))
    return removed


def compare_material_samples(before, after, path, normal, seed=314):
    bounds = before.bounding_box
    low = np.array([bounds[f"{axis}_min"] for axis in "xyz"])
    high = np.array([bounds[f"{axis}_max"] for axis in "xyz"])
    points = np.random.default_rng(seed).uniform(low, high, (2500, 3))
    tool = path["metadata"]["tool"]
    removed = cutter_contains(points, path, normal, tool["diameter_mm"] / 2, tool["flute_length_mm"])
    solid = after._shape.val()
    inside = np.array([solid.isInside(tuple(point), 1e-8) for point in points])
    assert np.array_equal(~removed, inside), points[(~removed) != inside]


@pytest.mark.parametrize("face", FACES)
def test_small_facing_tool_removes_the_entire_actual_stock_face(face):
    before = Stock.rectangular(40, 30, 20)
    after = before.with_fixture(FixtureCatalog.get("kurt_dx6_vise")).new_setup("face", face_selector=face).face_mill(
        depth=1, tool=ToolSpec("flat_endmill", 4, flute_length=10))
    path, normal = path_of(after)
    axis = "XYZ".index(face[1])
    area = math.prod([value for index, value in enumerate((40, 30, 20)) if index != axis])
    assert before.volume - after.volume == pytest.approx(area)
    compare_material_samples(before, after, path, normal)


@pytest.mark.parametrize("face", FACES)
@pytest.mark.parametrize("corner_radius", [2, 3.5])
def test_nonsquare_pocket_path_matches_native_dimensions_and_corner_radius(face, corner_radius):
    before = Stock.rectangular(40, 30, 20)
    after = before.pocket(width=12, length=16, depth=3, corner_radius=corner_radius,
                         cx=1, cy=-1, face_selector=face,
                         tool=ToolSpec("flat_endmill", 4, flute_length=10))
    path, normal = path_of(after)
    expected = (12 * 16 - (4 - math.pi) * corner_radius ** 2) * 3
    assert before.volume - after.volume == pytest.approx(expected)
    compare_material_samples(before, after, path, normal)


def test_facing_path_uses_changed_material_bounds_and_preserves_remaining_cuts():
    before = Stock.rectangular(40, 30, 20).pocket(width=12, length=16, depth=3,
        corner_radius=2, tool=ToolSpec("flat_endmill", 4, flute_length=10))
    after = before.face_mill(depth=1, tool=ToolSpec("flat_endmill", 4, flute_length=10))
    path, normal = path_of(after)
    assert path["metadata"]["incoming_bounds_mm"]["z_max"] == 10
    assert min(move["position"][2] for move in path["moves"]) == 9
    # Compare only points in the incoming material; the pocket remains empty.
    rng = np.random.default_rng(118)
    points = rng.uniform([-20, -15, -10], [20, 15, 10], (3000, 3))
    points = np.array([p for p in points if before._shape.val().isInside(tuple(p), 1e-8)])
    removed = cutter_contains(points, path, normal, 2, 10)
    inside = np.array([after._shape.val().isInside(tuple(p), 1e-8) for p in points])
    assert np.array_equal(~removed, inside)


def test_facing_preserves_translated_stock_coordinates():
    shape = cq.Workplane("XY").box(40, 30, 20).translate((17, -8, 6))
    before = Stock(_shape=shape, _material="aluminum_6061", _plan=ProcessPlan(),
                   _stock_dims={"length": 40, "width": 30, "height": 20})
    after = before.face_mill(depth=1, tool=ToolSpec("flat_endmill", 4, flute_length=10))
    path, normal = path_of(after)
    assert path["metadata"]["incoming_bounds_mm"]["x_min"] == pytest.approx(-3)
    assert path["metadata"]["incoming_bounds_mm"]["z_max"] == pytest.approx(16)
    compare_material_samples(before, after, path, normal)


def test_pocket_base_height_binds_the_floor_to_the_stock_datum():
    before = Stock.rectangular(40, 30, 10)
    after = before.pocket(width=12, length=16, depth=10, base_height=6, corner_radius=2,
                         tool=ToolSpec("flat_endmill", 4, flute_length=10))
    path, normal = path_of(after)
    assert min(move["position"][2] for move in path["moves"]) == 1
    compare_material_samples(before, after, path, normal)


def test_buried_pocket_has_no_face_toolpath():
    after = Stock.rectangular(40, 30, 10).pocket(width=12, length=16, depth=2,
        base_height=2, corner_radius=2, tool=ToolSpec("flat_endmill", 4, flute_length=10))
    record = after.process_plan()["operations"][-1]
    assert record["toolpath_status"] == "unavailable"
    assert "access face" in record["toolpath_reason"]


def test_unbound_facing_path_cannot_claim_actual_stock_coverage():
    assert "incoming_bounds_mm" not in FaceMillOp(tool=ToolSpec("flat_endmill", 4)).to_toolpath().metadata


@pytest.mark.parametrize("face", FACES)
def test_operation_video_uses_stock_positions_and_fixed_tool_orientation(face):
    from src.subcad.operation_video import _operation_moves, _is_cutting_move, _pose_from_payload
    after = Stock.rectangular(40, 30, 20).pocket(width=12, length=16, depth=3,
        corner_radius=2, face_selector=face, tool=ToolSpec("flat_endmill", 4, flute_length=10))
    path, normal = path_of(after)
    record = after.process_plan()["operations"][-1]
    moves = _operation_moves(record, {"z_max": 10})
    for actual, expected in zip(moves, path["moves"]):
        np.testing.assert_allclose(actual["position"], expected["position"], atol=1e-6)
        pose = _pose_from_payload(actual["pose"])
        np.testing.assert_allclose(pose.tool_axis, normal, atol=1e-6)
        assert _is_cutting_move(actual, actual["position"]) == (actual["type"] in {"feed", "plunge"})


@pytest.mark.parametrize("face", FACES)
def test_nonsquare_pocket_swept_cutter_matches_native_brep(face):
    """Use independent exact cylinders and bridge prisms, without grid sampling."""
    before = Stock.rectangular(40, 30, 10)
    after = before.pocket(width=16, length=10, depth=2, corner_radius=2, face_selector=face,
                         tool=ToolSpec("flat_endmill", 4, flute_length=10))
    path, normal = path_of(after)
    n = cq.Vector(*normal)
    remaining = before._shape.val()
    for a, b in cutting_segments(path):
        d = b - a
        dz = float(d @ normal)
        lateral = d - dz * normal
        length = float(np.linalg.norm(lateral))
        bottom = a if dz >= 0 else b
        cutter = cq.Solid.makeCylinder(2, 10 + abs(dz), cq.Vector(*bottom), n)
        if length > 1e-9:
            assert abs(dz) < 1e-9
            end = cq.Solid.makeCylinder(2, 10, cq.Vector(*b), n)
            plane = cq.Plane(origin=tuple(a), xDir=tuple(lateral / length), normal=tuple(normal))
            bridge = cq.Workplane(plane).center(length / 2, 0).rect(length, 4).extrude(10).val()
            cutter = cutter.fuse(end, bridge)
        remaining = remaining.cut(cutter)
    target = after._shape.val()
    assert remaining.cut(target).Volume() < 1e-7
    assert target.cut(remaining).Volume() < 1e-7
