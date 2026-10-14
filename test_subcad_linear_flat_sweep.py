"""Native cylindrical sweeps match continuous cutter membership and volume."""
import math
from itertools import product

import pytest

cq = pytest.importorskip("cadquery")
from src.subcad import ToolSpec
from src.subcad.milling_replay import _linear_flat_sweep, replay_pass
from src.subcad.toolpath import Toolpath, ToolpathMove


def continuous_cylinder_contains(point, delta, radius, length):
    x, y, z = point
    dx, dy, dz = delta
    low, high = 0.0, 1.0
    a, b, c = dx*dx+dy*dy, -2*(x*dx+y*dy), x*x+y*y-radius*radius
    if a == 0:
        if c > 0:
            return False
    else:
        discriminant = b*b-4*a*c
        if discriminant < 0:
            return False
        low, high = max(low, (-b-math.sqrt(discriminant))/(2*a)), min(high, (-b+math.sqrt(discriminant))/(2*a))
    if dz == 0:
        if z < 0 or z > length:
            return False
    else:
        interval = sorted((z/dz, (z-length)/dz))
        low, high = max(low, interval[0]), min(high, interval[1])
    return low <= high


@pytest.mark.parametrize("delta", [(1,2,-2), (2,1,2), (-1,-2,0.5), (0,0,2), (2,0,0), (0,0,0)])
def test_exact_flat_sweep_matches_a_continuous_cylinder_in_all_move_directions(delta):
    tool = ToolSpec("flat_endmill", 2, flute_length=4, stickout_mm=6)
    start = (20, -15, 6)
    shape = _linear_flat_sweep(tool, start, tuple(start[index]+delta[index] for index in range(3)))
    assert shape.isValid() and len(shape.Solids()) == 1
    expected = math.pi*(tool.radius**2)*(tool.flute_length+abs(delta[2])) + 2*tool.radius*tool.flute_length*math.hypot(delta[0],delta[1])
    assert shape.Volume() == pytest.approx(expected, abs=1e-6)
    for point in product((-1.13,-0.47,0.21,0.89,1.57,2.25), (-1.19,-0.51,0.17,0.85,1.53,2.21), (-1.91,-0.63,0.65,1.93,3.21,4.49,5.77)):
        expected_inside = continuous_cylinder_contains(point,delta,tool.radius,tool.flute_length)
        assert shape.isInside(cq.Vector(*(start[index]+point[index] for index in range(3))),1e-7) == expected_inside, point


def test_flat_sloping_replay_keeps_the_same_geometry_at_different_sampling_intervals():
    incoming = cq.Workplane("XY").box(8,8,6)
    tool = ToolSpec("flat_endmill",2,flute_length=6,stickout_mm=8)
    path = Toolpath("surface_mill",metadata={"coordinate_system":"part_mm","tool_axis":[0,0,1]},moves=[
        ToolpathMove("rapid",(-1,0,5)),ToolpathMove("plunge",(-1,0,2)),ToolpathMove("feed",(1,0,0)),ToolpathMove("retract",(1,0,5))])
    fine, fine_report = replay_pass(incoming,path,tool,sample_interval_mm=0.01)
    coarse, coarse_report = replay_pass(incoming,path,tool,sample_interval_mm=1)
    assert fine.val().cut(coarse.val()).Volume() < 1e-6
    assert coarse.val().cut(fine.val()).Volume() < 1e-6
    assert incoming.val().Volume() > fine.val().Volume()
    for report in (fine_report,coarse_report):
        assert report["sampled_poses"] == 0 and report["sample_interval_mm"] is None
        assert report["geometry_fidelity"] == "exact_cutter_sweep" and report["manufacturing_ready"] is False
