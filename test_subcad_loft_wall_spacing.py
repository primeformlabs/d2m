"""Preview accuracy must include the external part's curvature and actual cutter."""
import math

import cadquery as cq
import pytest

from src.subcad import Stock, ToolSpec
from src.subcad.loft_milling import _wall_paths
from src.subcad.milling_replay import replay_pass


@pytest.mark.parametrize("radius,cutter_radius,accuracy", [(3,5,.1),(2,2,.1),(1,5,.1),(20,5,.2),(3,.5,.1)])
def test_external_circular_spacing_bounds_the_selected_envelopes(radius,cutter_radius,accuracy):
    sections=[{"type":"circle","z_mm":0,"cx":0,"cy":0,"diameter":radius*2},
        {"type":"circle","z_mm":2,"cx":0,"cy":0,"diameter":2*(radius-min(1,radius/2))}]
    paths=_wall_paths(sections,tool=ToolSpec("flat_endmill",cutter_radius*2),tolerance=accuracy,safe_z=8,feed=100,rpm=1000)
    start,following=[path.moves[2].position for path in paths[:2]]
    midpoint_angle=math.atan2(following[1],following[0])/2
    rho=radius+cutter_radius
    measured=rho*math.cos(midpoint_angle)-math.sqrt(cutter_radius**2-(rho*math.sin(midpoint_angle))**2)-radius
    assert 0 <= measured <= accuracy
    assert paths[0].metadata["wall_spacing_error_bound_mm"] == pytest.approx(measured,abs=1e-10)
    assert paths[0].metadata["preview_accuracy_mm"] == accuracy


def test_native_wall_cuts_the_declared_accuracy_without_removing_required_material():
    origin=(20,-15,10)
    stock=Stock.rectangular(8,8,4)
    incoming=stock._shape.translate(origin)
    hole=cq.Workplane("XY",origin=(origin[0],origin[1],origin[2]-3)).circle(.3).extrude(6)
    incoming=incoming.cut(hole)
    initial_volume=incoming.val().Volume()
    sections=[{"type":"circle","z_mm":10,"cx":20,"cy":-15,"diameter":6},
        {"type":"circle","z_mm":12,"cx":20,"cy":-15,"diameter":4}]
    tool=ToolSpec("flat_endmill",10,flute_length=10)
    paths=_wall_paths(sections,tool=tool,tolerance=.1,safe_z=17,feed=100,rpm=1000)
    result=incoming
    for path in paths:
        result,_=replay_pass(result,path,tool,sample_interval_mm=path.metadata["independent_motion_interval_mm"])
    assert incoming.val().Volume() == pytest.approx(initial_volume)
    assert result.val().Volume() < initial_volume
    for z in (.02,.25,1,1.9):
        radius=3-z/2
        for index in range(72):
            angle=2*math.pi*(index+.5)/72
            def point(r):
                return cq.Vector(20+r*math.cos(angle),-15+r*math.sin(angle),10+z)
            assert not result.val().isInside(point(radius+.101))
            assert result.val().isInside(point(radius-.01))
    assert not result.val().isInside(cq.Vector(20,-15,11))
    assert result.val().isInside(cq.Vector(23.5,-11.5,9))


def test_path_budget_does_not_relax_the_preview_accuracy():
    sections=[{"type":"circle","z_mm":0,"cx":0,"cy":0,"diameter":6},
        {"type":"circle","z_mm":2,"cx":0,"cy":0,"diameter":4}]
    with pytest.raises(ValueError,match="finishing-path limit"):
        _wall_paths(sections,tool=ToolSpec("flat_endmill",10),tolerance=1e-6,safe_z=8,feed=100,rpm=1000)


def test_mixed_sections_keep_the_geometry_reason():
    from src.subcad.loft_milling import loft_sections
    sections=[{"type":"circle","z_mm":0,"diameter":6},
        {"type":"circle","z_mm":1,"diameter":4},
        {"type":"rectangle","z_mm":2,"length":2,"width":2}]
    with pytest.raises(ValueError,match="mixed sections"):
        loft_sections(sections)
