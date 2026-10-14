from copy import deepcopy
import math
import pytest
from src.subcad.face_motion import FaceMotionFrame
from src.subcad.toolpath import Toolpath, ToolpathMove


@pytest.mark.parametrize('face', ['>X', '<X', '>Y', '<Y', '>Z', '<Z'])
def test_absolute_part_frames_preserve_positions_and_the_independent_replay_contract(face):
    bounds = {f'{axis}_{end}': value for axis, low, high in [('x', 10, 30), ('y', -20, -5), ('z', 2, 9)]
              for end, value in [('min', low), ('max', high)]}
    frame = FaceMotionFrame(bounds, face)
    path = Toolpath('profile_contour', metadata={'coordinate_system': 'part_mm', 'tool_axis': frame.tool_axes['z_axis']},
                    moves=[ToolpathMove('rapid', (13, -16, 15)), ToolpathMove('feed', (14, -17, 4))])
    positions = deepcopy([move.position for move in path.moves])
    assert frame.bind_absolute(path) is path
    assert [move.position for move in path.moves] == positions
    assert path.metadata['coordinate_system'] == 'part_mm'
    assert path.metadata['tool_origin'] == 'cutting_tip'
    assert path.metadata['incoming_bounds_mm'] == bounds
    assert path.metadata['tool_frame'] == frame.tool_axes
    assert path.metadata['face_selector'] == face
    assert path.metadata['verification_status'] == 'not_verified'
    x, y, z = [path.metadata['tool_frame'][key] for key in ['x_axis', 'y_axis', 'z_axis']]
    assert [x[1]*y[2]-x[2]*y[1], x[2]*y[0]-x[0]*y[2], x[0]*y[1]-x[1]*y[0]] == z


@pytest.mark.parametrize('metadata', [
    {'coordinate_system': 'face_local_mm', 'tool_axis': [0, 0, 1]},
    {'coordinate_system': 'part_mm', 'tool_axis': [0, 0, -1]},
    {'coordinate_system': 'part_mm', 'tool_axis': [0, 0, 2]},
])
def test_absolute_binding_rejects_unknown_or_different_frames(metadata):
    frame = FaceMotionFrame({f'{axis}_{end}': value for axis in 'xyz' for end, value in [('min', -1), ('max', 1)]}, '>Z')
    with pytest.raises(ValueError, match='matching part coordinates'):
        frame.bind_absolute(Toolpath('surface_mill', metadata=metadata))


def test_absolute_binding_rejects_nonfinite_positions():
    frame = FaceMotionFrame({f'{axis}_{end}': value for axis in 'xyz' for end, value in [('min', -1), ('max', 1)]}, '>Z')
    path = Toolpath('surface_mill', metadata={'coordinate_system': 'part_mm', 'tool_axis': [0, 0, 1]},
                    moves=[ToolpathMove('feed', (0, 0, math.nan))])
    with pytest.raises(ValueError, match='finite linear'):
        frame.bind_absolute(path)
