import math

import cadquery as cq
import pytest

from src.subcad import Stock, compare_nominal_solids, load_nominal_step

IDENTITY = [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]


def plate(x):
    return cq.Workplane("XY").box(12,12,4).faces(">Z").workplane().pushPoints([(x,0)]).hole(2)


def test_equal_volume_and_envelope_do_not_hide_a_misplaced_hole():
    target, result = plate(-3), plate(3)
    assert target.val().Volume() == pytest.approx(result.val().Volume())
    before = result.val().Volume()
    comparison = compare_nominal_solids(result, target, target_to_part_matrix=IDENTITY)
    assert comparison.report["status"] == "differences"
    assert comparison.report["missing_material_mm3"] == pytest.approx(math.pi * 4, abs=1e-6)
    assert comparison.report["remaining_stock_mm3"] == pytest.approx(math.pi * 4, abs=1e-6)
    assert comparison.missing_material.isInside(cq.Vector(3,0,0))
    assert comparison.remaining_stock.isInside(cq.Vector(-3,0,0))
    assert result.val().Volume() == before
    assert comparison.report["tolerance_verified"] is False


def test_same_actual_solid_has_no_resolved_difference_and_no_manufacturing_claim():
    target = plate(3)
    report = compare_nominal_solids(target, target, target_to_part_matrix=IDENTITY)
    assert report.report["status"] == "no_resolved_difference"
    assert report.missing_material is None and report.remaining_stock is None
    assert report.report["manufacturing_ready"] is False


def test_step_geometry_uses_its_explicit_datum_and_preserves_the_original_target(tmp_path):
    target = plate(3).translate((20,-15,10))
    path = tmp_path / "required.step"
    cq.exporters.export(target, str(path))
    matrix = list(IDENTITY); matrix[3],matrix[7],matrix[11] = -20,15,-10
    comparison = compare_nominal_solids(plate(3), path, target_to_part_matrix=matrix)
    assert comparison.report["status"] == "no_resolved_difference"
    assert load_nominal_step(path).BoundingBox().xmin == pytest.approx(14)
    assert comparison.report["target_bounds"]["x_min"] == pytest.approx(-6)


def test_stock_candidates_use_their_actual_remaining_shape():
    candidate = Stock.rectangular(12,12,4)
    comparison = compare_nominal_solids(candidate, plate(3), target_to_part_matrix=IDENTITY)
    assert comparison.report["remaining_stock_mm3"] == pytest.approx(math.pi * 4, abs=1e-6)
    assert comparison.report["missing_material_mm3"] == pytest.approx(0)


def test_declared_rotation_matches_positions_without_inferred_alignment():
    target = plate(3)
    candidate = cq.Workplane("XY").box(12,12,4).faces(">Z").workplane().pushPoints([(0,3)]).hole(2)
    rotation = [0,-1,0,0,1,0,0,0,0,0,1,0,0,0,0,1]
    report = compare_nominal_solids(candidate, target, target_to_part_matrix=rotation)
    assert report.report["status"] == "no_resolved_difference"
    assert compare_nominal_solids(candidate, target, target_to_part_matrix=IDENTITY).report["status"] == "differences"


@pytest.mark.parametrize("matrix", [None, [1]*15, [1]*16,
    [2,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1], [-1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],
    [1,0,0,float('nan'),0,1,0,0,0,0,1,0,0,0,0,1]])
def test_invalid_or_scaled_datum_cannot_supply_target_proof(matrix):
    with pytest.raises(ValueError):
        compare_nominal_solids(plate(3), plate(3), target_to_part_matrix=matrix)


def test_whole_assembly_or_open_surfaces_cannot_supply_one_part_target(tmp_path):
    assembly = cq.Compound.makeCompound([cq.Workplane("XY").box(2,2,2).val(),
        cq.Workplane("XY").box(2,2,2).translate((5,0,0)).val()])
    path = tmp_path / "assembly.step"
    cq.exporters.export(assembly, str(path))
    with pytest.raises(ValueError, match="one closed part solid"):
        load_nominal_step(path)
    with pytest.raises(ValueError, match="one closed part solid"):
        compare_nominal_solids(cq.Workplane("XY").rect(4,4).val(), plate(3), target_to_part_matrix=IDENTITY)
