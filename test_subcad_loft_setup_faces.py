"""Lofts preserve actual material, section coordinates, and motion on six faces."""
from copy import deepcopy

import cadquery as cq
import numpy as np
import pytest

from src.subcad import Stock, ToolSpec, replay_milling_independently
from src.subcad.profile_milling import _setup_plane
from src.subcad.toolpath import Toolpath


FACES=[(">Z",[0,0,1],lambda u,v,w:(u,v,w)),("<Z",[0,0,-1],lambda u,v,w:(u,v,-w)),
    (">X",[1,0,0],lambda u,v,w:(w,u,v)),("<X",[-1,0,0],lambda u,v,w:(-w,u,v)),
    (">Y",[0,1,0],lambda u,v,w:(u,w,v)),("<Y",[0,-1,0],lambda u,v,w:(u,-w,v))]


def incoming(face, point):
    stock=Stock.rectangular(8,8,4)
    plane,mirror=_setup_plane(face)
    local=stock._shape.translate((1,-1*mirror,7))
    hole=cq.Solid.makeCylinder(.3,6,cq.Vector(1,-1*mirror,4))
    local=local.cut(hole)
    stock._shape=stock._shape.newObject([local.val().transformShape(plane.rG)])
    return stock


@pytest.mark.parametrize("face,axis,point",FACES)
@pytest.mark.parametrize("kind",["circle","rectangle"])
def test_loft_faces_keep_translated_holes_and_world_motion(face,axis,point,kind):
    stock=incoming(face,point)
    profiles=[{"type":"circle","z_mm":7,"cx":1,"cy":-1,"diameter":4},
        {"type":"circle","z_mm":9,"cx":1,"cy":-1,"diameter":2}]
    if kind=="rectangle":
        profiles=[{"type":"rectangle","z_mm":7,"cx":1,"cy":-1,"length":4,"width":2},
            {"type":"rectangle","z_mm":9,"cx":1,"cy":-1,"length":2,"width":1}]
    original=deepcopy(profiles)
    before=stock.volume
    result=stock.loft_mill(profiles,tool=ToolSpec("flat_endmill",4),face_selector=face,tolerance_mm=.5)
    assert profiles==original and stock.volume==pytest.approx(before)
    def inside(u,v,w):return result._shape.val().isInside(cq.Vector(*point(u,v,w)))
    assert not inside(1,-1,8)
    assert inside(2,-1,8)
    assert not inside(4,-1,8)
    assert inside(4,2,6)
    record=result.process_plan()["operations"][-1]
    assert record["toolpath"]["metadata"]["tool_axis"]==axis
    assert record["toolpath"]["metadata"]["coordinate_system"]=="part_mm"
    assert record["operation_target_comparison"]["overcut_volume_mm3"]<1e-6
    assert record["operation_target_comparison"]["comparison_validated"]
    assert len(result._state_history)==len(record["material_passes"])+1
    assert result._shape.val().isSame(result._state_history[-1]["shape"].val())
    for state in result._state_history:
        assert state["shape"].val().isValid()
        assert not state["shape"].val().isInside(cq.Vector(*point(1,-1,8)))
    # A cutting move reaches the declared base plane along the actual spindle axis.
    moves=record["toolpath"]["moves"]
    cutting=[move["position"] for move in moves if move["type"]=="feed"]
    assert min(sum(p[i]*axis[i] for i in range(3)) for p in cutting)==pytest.approx(7)
    assert max(sum(p[i]*axis[i] for i in range(3)) for p in cutting)<=9+1e-7


@pytest.mark.parametrize("face,axis,point",FACES)
def test_world_loft_paths_replay_independently_on_each_setup_face(face,axis,point):
    stock=incoming(face,point)
    tool=ToolSpec("flat_endmill",4)
    profiles=[{"type":"circle","z_mm":7,"cx":1,"cy":-1,"diameter":4},
        {"type":"circle","z_mm":9,"cx":1,"cy":-1,"diameter":2}]
    result=stock.loft_mill(profiles,tool=tool,face_selector=face,tolerance_mm=.5)
    record=result.process_plan()["operations"][-1]
    model,report=replay_milling_independently(stock,Toolpath.from_dict(record["toolpath"]),tool,
        resolution_mm=.5,target=result)
    assert not model.get_material_at(np.array(point(3.25,-.75,8.25)))
    assert model.get_material_at(np.array(point(1.75,-.75,8.25)))
    assert report["target_comparison"]["status"]=="sampled_match",report["target_comparison"]
    assert not report["manufacturing_ready"]


def test_side_loft_preserves_actual_turned_stock_and_its_operation_history():
    stock=Stock.round_bar(4,8,axis="X",origin=(7,1,-1)).turn_od(7,4)
    before=stock.volume
    preceding=deepcopy(stock.process_plan()["operations"])
    result=stock.loft_mill([{"type":"circle","z_mm":7,"cx":1,"cy":-1,"diameter":4},
        {"type":"circle","z_mm":9,"cx":1,"cy":-1,"diameter":2}],
        face_selector=">X",tool=ToolSpec("flat_endmill",4),tolerance_mm=.5)
    assert stock.volume==pytest.approx(before)
    assert result.volume<before
    assert result.process_plan()["operations"][:-1]==preceding
    assert result._shape.val().isInside(cq.Vector(6,3,-1))
    assert not result._shape.val().isInside(cq.Vector(8,4,-1))


def test_comparison_keeps_real_overcut_and_remaining_material_after_translation():
    from src.subcad.loft_milling import _comparison_differences
    incoming=cq.Solid.makeBox(8,8,4,cq.Vector(20,-15,6))
    required=cq.Solid.makeBox(4,4,2,cq.Vector(22,-13,7))
    missing=cq.Solid.makeBox(1,1,1,cq.Vector(22,-13,7))
    overcut,remaining=_comparison_differences(incoming,required,required.cut(missing))
    assert overcut==pytest.approx(1) and remaining==pytest.approx(0,abs=1e-7)
    excess=cq.Solid.makeBox(1,1,1,cq.Vector(21,-13,7))
    overcut,remaining=_comparison_differences(incoming,required,required.fuse(excess))
    assert overcut==pytest.approx(0,abs=1e-7) and remaining==pytest.approx(1)
