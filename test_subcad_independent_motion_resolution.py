import pytest
from src.subcad import Stock, ToolSpec, replay_milling_independently
from src.subcad.toolpath import Toolpath, ToolpathMove


def path(interval):
    return Toolpath("loft_mill",tool_type="flat_endmill",tool_diameter_mm=2,
        metadata={"coordinate_system":"part_mm","tool_axis":[0,0,1],
            "material_passes":[{"sample_interval_mm":interval}]},
        moves=[ToolpathMove("rapid",(-1,0,4)),ToolpathMove("plunge",(-1,0,1)),ToolpathMove("feed",(1,0,0))])


def test_independent_motion_keeps_the_retained_resolution_with_a_coarser_spatial_grid():
    _,report=replay_milling_independently(Stock.rectangular(4,4,4),path(0.05),ToolSpec("flat_endmill",2),resolution_mm=0.5)
    assert report["motion_resolution_mm"]==0.05
    assert report["resolution_mm"]==0.5
    assert report["removed_volume_mm3"]>0
    assert not report["manufacturing_ready"]


@pytest.mark.parametrize("interval,reason",[(float("nan"),"finite"),(-0.1,"positive"),(1e-300,"pose limit")])
def test_independent_motion_rejects_invalid_or_unbounded_retained_intervals(interval,reason):
    with pytest.raises(ValueError,match=reason):
        replay_milling_independently(Stock.rectangular(4,4,4),path(interval),ToolSpec("flat_endmill",2),resolution_mm=0.5)
