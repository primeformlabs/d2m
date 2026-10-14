from copy import deepcopy
import pytest
import cadquery as cq
from src.subcad import Stock, ToolSpec, tool_inventory


def sections(kind="rectangle", *, x=0, y=0, z=0):
    dimensions = ({"length":4,"width":4}, {"length":2,"width":2}) if kind == "rectangle" else (
        {"diameter":4}, {"diameter":2})
    return [{"type":kind,"z_mm":z + index * 2,"cx":x,"cy":y,**dimension}
        for index,dimension in enumerate(dimensions)]


@pytest.mark.parametrize("kind",["rectangle","circle"])
def test_loft_cuts_actual_material_and_retains_compensated_roughing_and_finishing(kind):
    stock=Stock.rectangular(8,8,4)
    result=stock.loft_mill(sections(kind),tool=ToolSpec("flat_endmill",4),tolerance_mm=0.5)
    assert stock.volume==pytest.approx(256)
    assert result.volume<stock.volume
    assert result._shape.val().isInside(cq.Vector(0,0,1.8))
    assert not result._shape.val().isInside(cq.Vector(3,3,1.8))
    assert result._shape.val().isInside(cq.Vector(3,3,-1))
    record=result.process_plan()["operations"][-1]
    assert record["operation"]=="loft_mill"
    assert record["operation_target_comparison"]["overcut_volume_mm3"]<1e-6
    assert record["operation_target_comparison"]["remaining_target_material_mm3"]>0
    assert record["operation_target_comparison"]["remaining_target_material_mm3"] < record["operation_target_comparison"]["clearing_remaining_material_mm3"] / 2
    assert record["operation_target_comparison"]["wall_finishing_enabled"]
    assert record["verification_status"]=="not_verified"
    assert record["achieved_tolerance_estimate_mm"] is None
    assert record["cutter_compensation"]=="geometry_applied"
    assert {state["pass_role"] for state in record["material_passes"]}=={"roughing","finishing"}
    assert any(state.get("machining_phase")=="wall_finishing" for state in record["material_passes"])
    assert len(result._state_history)==len(record["material_passes"])+1
    assert all(state["shape"].val().isValid() for state in result._state_history)
    assert all(first["shape"].val().Volume()>=last["shape"].val().Volume()-1e-7
        for first,last in zip(result._state_history,result._state_history[1:]))
    moves=record["toolpath"]["moves"]
    for state in record["material_passes"]:
        assert 0<=state["move_start_index"]<state["move_end_index"]<=len(moves)
        assert not state["manufacturing_ready"]


def test_loft_preserves_translated_incoming_holes_and_uses_owned_tool_geometry():
    stock=Stock.rectangular(8,8,4)
    stock._shape=stock._shape.translate((20,-15,10))
    hole=cq.Workplane("XY",origin=(20,-15,7)).circle(0.4).extrude(6)
    stock._shape=stock._shape.cut(hole)
    before=stock.volume
    profiles=sections(x=20,y=-15,z=10)
    saved=deepcopy(profiles)
    cutter=ToolSpec("flat_endmill",4,flute_length=4,reach_mm=4,stickout_mm=5,
        catalog_id="OWNED-LOFT-4",physical_assembly={"owned":True})
    with tool_inventory([cutter],revision="owned-loft",strict=True):
        result=stock.loft_mill(profiles,tolerance_mm=0.5)
    assert stock.volume==pytest.approx(before)
    assert profiles==saved
    assert not result._shape.val().isInside(cq.Vector(20,-15,11))
    assert result._shape.val().isInside(cq.Vector(20.8,-15,11))
    record=result.process_plan()["operations"][-1]
    assert record["tool"]["catalog_id"]=="OWNED-LOFT-4"
    assert record["tool_selection"]["inventory_revision"]=="owned-loft"
    assert record["toolpath"]["metadata"]["coordinate_system"]=="part_mm"
    assert record["operation_target_comparison"]["overcut_volume_mm3"]<1e-6


@pytest.mark.parametrize("change,reason",[
    (lambda p:p[1].update(z_mm=0),"increase"),
    (lambda p:p[1].update(cx=1),"concentric"),
    (lambda p:p[1].update(length=5),"overhang"),
    (lambda p:p[1].update(type="triangle"),"section"),
    (lambda p:p[0].update(width=float("nan")),"finite"),
    (lambda p:p[0].update(z_mm=-2),"below"),
    (lambda p:p[1].update(z_mm=1),"top"),
    (lambda p:p[0].update(length=20),"bounds"),
    (lambda p:p[0].update(cx_mm=20),"unsupported geometry fields"),
    (lambda p:p[0].update(type=[]),"section"),
])
def test_loft_rejects_unreachable_missing_or_invalid_sections_without_changing_source(change,reason):
    stock=Stock.rectangular(8,8,4)
    profiles=sections()
    change(profiles)
    with pytest.raises(ValueError,match=reason):
        stock.loft_mill(profiles,tool=ToolSpec("flat_endmill",4),tolerance_mm=0.5)
    assert stock.volume==pytest.approx(256)


def test_loft_requires_owned_tools_and_rejects_unreachable_cutter_or_setup():
    with tool_inventory([],strict=True),pytest.raises(ValueError,match="flat_endmill"):
        Stock.rectangular(8,8,4).loft_mill(sections())
    with pytest.raises(ValueError,match="cannot reach"):
        Stock.rectangular(8,8,4).loft_mill(sections(),tool=ToolSpec("flat_endmill",4,flute_length=1))
    with pytest.raises(ValueError,match="positive Z"):
        Stock.rectangular(8,8,4).loft_mill(sections(),tool=ToolSpec("flat_endmill",4),face_selector=">X")


def test_loft_path_replays_independently_against_incoming_material():
    import numpy as np
    from src.subcad import replay_milling_independently
    from src.subcad.toolpath import Toolpath
    incoming=Stock.round_bar(4,8).turn_od(7,4)
    tool=ToolSpec("flat_endmill",4)
    result=incoming.loft_mill(sections("circle"),tool=tool,tolerance_mm=0.5)
    record=result.process_plan()["operations"][-1]
    path=Toolpath.from_dict(record["toolpath"])
    model,report=replay_milling_independently(incoming,path,tool,resolution_mm=0.5,target=result)
    assert model.get_material_at(np.array([0.25,0.25,1.25]))
    assert not model.get_material_at(np.array([2.75,0.25,1.25]))
    assert report["target_comparison"]["status"]=="sampled_match",report["target_comparison"]
    assert report["motion_resolution_mm"]<=0.125
    assert not report["manufacturing_ready"] and not report["target_comparison"]["tolerance_verified"]
    assert incoming._shape.val().isInside(cq.Vector(2.75,0.25,1.25))


def test_loft_rejects_an_unbounded_preview_interval_before_native_replay():
    with pytest.raises(ValueError,match="depth-pass limit"):
        Stock.rectangular(8,8,4).loft_mill(sections(),tool=ToolSpec("flat_endmill",4),tolerance_mm=1e-320)


def test_wall_finishing_flag_and_path_budget_have_specific_reasons():
    from src.subcad.loft_milling import _wall_paths
    with pytest.raises(ValueError,match="true or false"):
        Stock.rectangular(8,8,4).loft_mill(sections(),tool=ToolSpec("flat_endmill",4),finish_walls="yes")
    with pytest.raises(ValueError,match="finishing.*limit"):
        _wall_paths(sections(),tool=ToolSpec("flat_endmill",4),tolerance=1e-8,safe_z=4,feed=100,rpm=1000)
