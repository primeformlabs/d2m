"""Actual sheet material, radiused bends, continuity, and explicit failures."""
from copy import deepcopy
import math

import cadquery as cq
import numpy as np
import pytest
import trimesh

from src.subcad.bending_material import replay_bending_material


def sheet():
    native = cq.Workplane("XY").box(40, 60, 2).faces(">Z").workplane().pushPoints(
        [(-10, -20), (10, -20), (-10, 20), (10, 20)]).hole(4)
    vertices, faces = native.val().tessellate(0.05)
    return trimesh.Trimesh(vertices=[[v.x, -v.z, v.y] for v in vertices], faces=faces)


def plan(*, balanced=False):
    return {"flat_blank": {"length_mm": 40, "width_mm": 60, "thickness_mm": 2}, "steps": [
        {"id": f"bend-{index + 1}", "order": index + 1, "angle_deg": 90, "signed_fold_angle_deg": signed,
         "inside_radius_mm": 2, "bend_line": {"start_mm": {"x": start, "y": 0, "z": z},
             "end_mm": {"x": -start, "y": 0, "z": z}, "normal_mm": {"x": 0, "y": 1, "z": 0}},
         "moving_side": "positive", "support_mode": "balanced" if balanced else "fixed"}
        for index, (start, z, signed) in enumerate([(20, -10, 90), (-20, 10, -90)])]}


@pytest.mark.parametrize("balanced", [False, True])
def test_actual_sheet_holes_and_closed_material_survive_two_bends(balanced):
    source = sheet()
    original = source.vertices.copy()
    specification = plan(balanced=balanced)
    saved_plan = deepcopy(specification)
    replay = replay_bending_material(source, specification)
    assert len(replay.passes) == 2
    assert source.vertices == pytest.approx(original)
    assert specification == saved_plan
    assert replay.passes[0].before.volume == pytest.approx(40 * 60 * 2 - 4 * math.pi * 2**2 * 2, rel=0.001)
    assert replay.passes[1].before is replay.passes[0].after
    for operation in replay.passes:
        assert operation.after.is_watertight
        assert operation.after.is_winding_consistent
        assert operation.after.euler_number == source.euler_number
        assert operation.after.volume == pytest.approx(source.volume, rel=0.01)
        assert operation.report["manufacturing_ready"] is False
        assert operation.report["validation_status"] == "not_verified"
    assert np.ptp(replay.material.bounds[:, 1]) > np.ptp(source.bounds[:, 1]) * 5
    assert not np.array_equal(replay.passes[0].after.bounds, replay.passes[1].after.bounds)


@pytest.mark.parametrize("field,value", [("signed_fold_angle_deg", 45), ("inside_radius_mm", -1),
    ("moving_side", "guess"), ("support_mode", "guess"), ("order", 2), ("angle_deg", float("nan"))])
def test_incomplete_or_invalid_bend_intent_cannot_produce_a_result(field, value):
    specification = plan()
    specification["steps"][0][field] = value
    with pytest.raises(ValueError):
        replay_bending_material(sheet(), specification)


def test_open_or_wrong_frame_material_cannot_use_a_substitute_blank():
    source = sheet()
    source.faces = source.faces[:-1]
    with pytest.raises(ValueError, match="closed"):
        replay_bending_material(source, plan())
    source = sheet()
    source.apply_translation([0, 5, 0])
    with pytest.raises(ValueError, match="flat frame"):
        replay_bending_material(source, plan())


def test_partial_or_repeated_identity_plan_cannot_produce_a_result():
    specification = plan()
    specification["partial_sequence"] = True
    with pytest.raises(ValueError, match="complete"):
        replay_bending_material(sheet(), specification)
    specification = plan()
    specification["steps"][1]["id"] = specification["steps"][0]["id"]
    with pytest.raises(ValueError, match="unique"):
        replay_bending_material(sheet(), specification)


def test_an_absent_bend_region_or_impossible_radius_cannot_return_unchanged_material():
    specification = plan()
    specification["steps"][0]["inside_radius_mm"] = 100
    with pytest.raises(ValueError, match="insufficient material"):
        replay_bending_material(sheet(), specification)
    specification = plan()
    for key in ("start_mm", "end_mm"):
        specification["steps"][0]["bend_line"][key]["z"] = 50
    with pytest.raises(ValueError, match="insufficient material"):
        replay_bending_material(sheet(), specification)


def test_classifier_sheet_face_direction_cannot_disagree_with_the_saved_fold():
    specification = plan()
    specification["steps"][0].update(direction_convention="sheet_face", direction="up", signed_fold_angle_deg=90)
    with pytest.raises(ValueError, match="contradicts"):
        replay_bending_material(sheet(), specification)
