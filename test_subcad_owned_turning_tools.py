"""Recorded Turning tools must survive native cuts and reject incompatible intent."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy

import pytest

from src.subcad import Stock, TurningToolSpec, turning_tool_inventory
from src.subcad.tool_selection import ToolUnavailableError


def tool(role='face', key='face-1', **values):
    return TurningToolSpec(key, role, 'CNMG120408', 'PCLNR2525M12', 0.8, 'R', 3,
        {'assembly_id': key, 'holder_id': 'holder-1', 'owned': True,
         'machine': {'machine_id': 'lathe-1'}, 'station': 2, 'offset_number': 12,
         'geometry_status': 'not_recorded'}, **values)


def test_recorded_assembly_survives_native_material_removal_without_milling_defaults():
    original = Stock.round_bar(30, 20)
    with turning_tool_inventory([tool()], revision='inventory-1'):
        result = original.turn_face(14)
    operation = result.process_plan()['operations'][0]
    assert result.volume < original.volume
    assert operation['tool_selection']['inventory_revision'] == 'inventory-1'
    assert operation['tool']['assembly']['station'] == 2
    assert operation['tool']['assembly']['offset_number'] == 12
    assert operation['tool']['insert_iso'] == 'CNMG120408'
    assert 'flute_length_mm' not in operation['tool']
    assert operation['tool'].get('diameter_mm') is None
    assert operation['tool_diameter_mm'] is None
    assert not operation['requires_owned_tool']
    assert operation['verification_status'] == 'not_verified'


def test_empty_inventory_never_substitutes_a_generic_turning_insert():
    with turning_tool_inventory([], revision='empty'):
        with pytest.raises(ToolUnavailableError) as error:
            Stock.round_bar(30, 20).turn_face(14)
    assert error.value.result.inventory_revision == 'empty'
    assert error.value.result.selected_tool is None


@pytest.mark.parametrize('change', ['role', 'owned', 'geometry', 'explicit'])
def test_wrong_role_unowned_invalid_and_forged_tools_fail(change):
    candidate = tool()
    explicit = None
    if change == 'role': candidate.role = 'od_finish'
    elif change == 'owned': candidate.physical_assembly['owned'] = False
    elif change == 'geometry': candidate.nose_radius_mm = float('nan')
    else:
        explicit = deepcopy(candidate)
        explicit.physical_assembly['station'] = 4
    with turning_tool_inventory([candidate], revision='current'):
        with pytest.raises(ToolUnavailableError):
            Stock.round_bar(30, 20).turn_face(14, tool=explicit)


@pytest.mark.parametrize('operation,candidate,reason', [
    (lambda s: s.turn_id(8, 12), tool('bore', min_bore_mm=10, max_depth_mm=20), 'minimum_bore_exceeded'),
    (lambda s: s.turn_id(12, 15), tool('bore', min_bore_mm=8, max_depth_mm=10), 'cutting_depth_exceeded'),
    (lambda s: s.turn_groove(2, 1, 0), tool('groove_od', width_mm=3, max_depth_mm=4), 'groove_tool_too_wide'),
    (lambda s: s.turn_thread(20, 1.5, 10), tool('thread_od', pitch_range=(2, 3), profile_angle=60), 'thread_pitch_not_supported'),
    (lambda s: s.turn_thread(20, 1.5, 10), tool('thread_od', pitch_range=(1, 2), profile_angle=55), 'thread_angle_not_supported'),
])
def test_recorded_cutting_limits_reject_incompatible_operations(operation, candidate, reason):
    with turning_tool_inventory([candidate], revision='limits'):
        with pytest.raises(ToolUnavailableError) as error:
            operation(Stock.round_bar(30, 20))
    assert reason in error.value.result.rejected_tools[0]['reasons']


def test_rough_and_finish_roles_keep_separate_tools_and_inventory_scopes():
    def cut(role):
        with turning_tool_inventory([tool(role, role)], revision=role, role=role):
            result = Stock.round_bar(30, 20).turn_profile([(8, -15), (8, 15)])
        operation = result.process_plan()['operations'][0]
        return operation['tool']['catalog_id'], operation['tool_selection']['inventory_revision']
    with ThreadPoolExecutor(max_workers=2) as pool:
        assert list(pool.map(cut, ['od_rough', 'od_finish'])) == [('od_rough', 'od_rough'), ('od_finish', 'od_finish')]
