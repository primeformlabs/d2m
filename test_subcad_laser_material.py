import math
import cadquery as cq
import pytest
import json
from pathlib import Path
from subcad import Stock, replay_laser_material, release_laser_part, laser_selection_region
from subcad.geometry import export_stl


def test_mesh_export_preserves_live_material_through_eight_circular_holes(tmp_path):
    fixture = json.loads((Path(__file__).parent / "test_fixtures/laser_u_channel_paths.json").read_text())
    stock = Stock.sheet(*fixture["sheet_mm"])
    material = stock._shape.translate((100, 100, 0))
    for index, operation in enumerate(fixture["paths"]):
        result = replay(material, operation["path_mm"], fixture["kerf_mm"])
        before = result.material.Volume()
        export_stl(cq.Workplane("XY").newObject([result.passes[0].after]), str(tmp_path / f"pierce_{index}.stl"))
        export_stl(cq.Workplane("XY").newObject([result.material]), str(tmp_path / f"cut_{index}.stl"))
        assert result.material.isValid()
        assert result.material.Volume() == pytest.approx(before, abs=1e-7)
        region = laser_selection_region(operation["hole_mm"], [], thickness_mm=2, bottom_z_mm=-1)
        released = release_laser_part(result.material, region, envelope_tolerance_mm=0.05)
        assert released.part.Volume() > 0
        assert released.remainder.isValid()
        material = released.remainder
    assert len(material.Solids()) == 1


def replay(source, path, kerf=0.4):
    return replay_laser_material(source, path, kerf_width_mm=kerf, units="mm", coordinate_system="stock_local_mm")


def test_native_beam_removes_a_capsule_without_filling_the_contour():
    source = Stock.sheet(20, 20, 2)
    initial = source.volume
    result = replay(source, [(-6, 0), (6, 0)])
    assert result.report["removed_volume_mm3"] == pytest.approx((12 * 0.4 + math.pi * 0.2**2) * 2, abs=1e-6)
    assert source.volume == pytest.approx(initial)
    assert len(result.passes) == 2
    assert result.passes[0].kind == "pierce"
    assert result.passes[1].before.Volume() == pytest.approx(result.passes[0].after.Volume())
    assert result.report["manufacturing_ready"] is False


def test_closed_path_retains_slug_and_sheet_until_explicit_release():
    source = Stock.sheet(20, 20, 2)
    result = replay(source, [(-5.2,-5.2), (5.2,-5.2), (5.2,5.2), (-5.2,5.2), (-5.2,-5.2)])
    assert len(result.material.Solids()) == 2
    target = cq.Workplane("XY").box(10, 10, 2)
    released = release_laser_part(result.material, target, envelope_tolerance_mm=1e-5)
    assert released.part.Volume() == pytest.approx(200, abs=1e-5)
    assert released.part.Volume() + released.remainder.Volume() == pytest.approx(result.material.Volume())
    assert released.report["geometry_source"] == "actual_cut_material_piece"
    assert source.volume == pytest.approx(800)


def test_replay_preserves_preceding_holes_and_translated_coordinates():
    native = cq.Workplane("XY").box(20,20,2, centered=(True,True,False)).faces(">Z").workplane().hole(2)
    native = native.translate((30,40,8))
    result = replay(native, [(24,44), (36,44)])
    assert result.material.BoundingBox().zmin == pytest.approx(8)
    hole_probe = cq.Solid.makeCylinder(0.9,2,cq.Vector(30,40,8))
    assert result.material.intersect(hole_probe).Volume() == pytest.approx(0,abs=1e-7)
    assert native.val().Volume() == pytest.approx(800 - math.pi * 2)


def test_kerf_changes_actual_removed_material():
    source = Stock.sheet(20,20,2)
    narrow = replay(source,[(-4,0),(4,0)],0.2)
    wide = replay(source,[(-4,0),(4,0)],0.6)
    assert wide.report["removed_volume_mm3"] > narrow.report["removed_volume_mm3"] * 2


@pytest.mark.parametrize("path,kerf", [([[0,0],[0,0]],0.2), ([[0,0],[float("nan"),1]],0.2),
    ([[0,0],[True,1]],0.2), ([[0,0],[1,1]],0), ([[0,0],[1,1]],True)])
def test_invalid_path_or_kerf_cannot_return_usable_material(path, kerf):
    with pytest.raises(ValueError):
        replay(Stock.sheet(20,20,2),path,kerf)


def test_release_rejects_an_open_cut_and_does_not_replace_stock_with_reference():
    result = replay(Stock.sheet(20,20,2),[(-5,-5),(5,-5),(5,5)])
    target = cq.Workplane("XY").box(10,10,2)
    with pytest.raises(ValueError,match="detached"):
        release_laser_part(result.material,target,envelope_tolerance_mm=0.01)


def test_air_moves_remain_explicit_without_a_material_removal_claim():
    result = replay(Stock.sheet(20,20,2),[(30,30),(40,30)])
    assert result.report["status"] == "no_material_removed"
    assert result.report["removed_volume_mm3"] == pytest.approx(0)


def test_declared_units_and_frame_are_required():
    with pytest.raises(ValueError,match="stock-local"):
        replay_laser_material(Stock.sheet(20,20,2),[(0,0),(1,1)],kerf_width_mm=0.2,
            units="inch",coordinate_system="part_mm")


def test_selection_region_preserves_declared_coordinates_and_holes():
    region = laser_selection_region([[10,20],[20,20],[20,30],[10,30]],
        [[[14,24],[16,24],[16,26],[14,26]]], thickness_mm=2, bottom_z_mm=8)
    assert region.Volume() == pytest.approx(192)
    assert region.BoundingBox().xmin == pytest.approx(10)
    assert region.BoundingBox().zmin == pytest.approx(8)
    with pytest.raises(ValueError,match="outside"):
        laser_selection_region([[10,20],[20,20],[20,30],[10,30]],
            [[[30,30],[31,30],[31,31],[30,31]]], thickness_mm=2,bottom_z_mm=8)
