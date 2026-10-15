"""Select recorded Turning assemblies without inventing Milling dimensions."""
from contextlib import contextmanager
from contextvars import ContextVar
from copy import deepcopy
from dataclasses import dataclass, field
import math

from .tool_selection import ToolRequirement, ToolSelectionResult, ToolUnavailableError


@dataclass
class TurningToolSpec:
    catalog_id: str
    role: str
    insert_iso: str
    holder_iso: str
    nose_radius_mm: float
    hand: str
    tip_direction: int
    physical_assembly: dict
    min_bore_mm: float | None = None
    width_mm: float | None = None
    max_depth_mm: float | None = None
    pitch_range: tuple | None = None
    profile_angle: float | None = None
    preferred_feeds_speeds: dict = field(default_factory=dict)
    safe_limits: dict = field(default_factory=dict)
    inventory_count: int = 1
    tool_type: str = field(default='turning_insert', init=False)
    # The common path interface uses a numeric diameter. An insert has no rotary cutter diameter.
    diameter: float = field(default=0.0, init=False, repr=False)

    def to_dict(self):
        return {key: deepcopy(getattr(self, key)) for key in (
            'catalog_id', 'tool_type', 'role', 'insert_iso', 'holder_iso', 'nose_radius_mm',
            'hand', 'tip_direction', 'min_bore_mm', 'width_mm', 'max_depth_mm', 'pitch_range',
            'profile_angle', 'safe_limits', 'preferred_feeds_speeds', 'inventory_count')} | {
                'assembly': deepcopy(self.physical_assembly), 'label': self.catalog_id}


_scope = ContextVar('subcad_turning_inventory', default=None)
ROLES = {'face', 'od_rough', 'od_finish', 'bore', 'groove_od', 'groove_id', 'thread_od', 'thread_id', 'part_off'}


def _number(value, *, zero=False):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
        and math.isfinite(value) and (value >= 0 if zero else value > 0))


@contextmanager
def turning_tool_inventory(tools, *, revision, role=None):
    """Limit each operation to an isolated, recorded inventory revision."""
    if not isinstance(revision, str) or not revision:
        raise ValueError('The Turning inventory needs a revision.')
    if role is not None and role not in ROLES:
        raise ValueError('The Turning inventory has an unsupported operation role.')
    snapshot = tuple(deepcopy(tool) for tool in tools)
    ids = [tool.catalog_id for tool in snapshot]
    if any(not key for key in ids) or len(ids) != len(set(ids)):
        raise ValueError('Turning assemblies need distinct inventory IDs.')
    token = _scope.set((snapshot, revision, role))
    try:
        yield
    finally:
        _scope.reset(token)


def select_turning_tool(operation, explicit=None):
    scope = _scope.get()
    if scope is None:
        return explicit, None
    inventory, revision, override = scope
    name = operation.operation_name
    role = override or {'turn_face': 'face', 'turn_profile': 'od_finish', 'turn_od': 'od_finish',
        'turn_id': 'bore', 'turn_groove': 'groove_id' if getattr(operation, 'internal', False) else 'groove_od',
        'turn_thread': 'thread_id' if getattr(operation, 'internal', False) else 'thread_od'}.get(name)
    result = ToolSelectionResult(ToolRequirement('turning_insert', feature=role or name,
        reason='Use a recorded Turning assembly for this operation.'), inventory_revision=revision)
    for tool in sorted(inventory, key=lambda item: item.catalog_id):
        reasons = []
        if tool.role != role:
            reasons.append('wrong_turning_role')
        if tool.inventory_count <= 0 or tool.physical_assembly.get('owned') is not True:
            reasons.append('not_owned')
        if not tool.insert_iso or not tool.holder_iso or tool.hand not in {'R', 'L', 'N'}:
            reasons.append('incomplete_insert_or_holder')
        if (not _number(tool.nose_radius_mm, zero=True) or isinstance(tool.tip_direction, bool)
                or not isinstance(tool.tip_direction, int) or tool.tip_direction not in range(1, 10)):
            reasons.append('invalid_insert_geometry')
        if role in {'bore', 'groove_id', 'thread_id'}:
            diameter = getattr(operation, 'diameter', None)
            if not _number(tool.min_bore_mm) or not _number(diameter) or tool.min_bore_mm > diameter:
                reasons.append('minimum_bore_exceeded')
        if role in {'bore', 'groove_id', 'groove_od', 'part_off'}:
            depth = getattr(operation, 'length', None) if role == 'bore' else getattr(operation, 'depth', None)
            if not _number(tool.max_depth_mm) or not _number(depth) or tool.max_depth_mm < depth:
                reasons.append('cutting_depth_exceeded')
        if role in {'groove_id', 'groove_od', 'part_off'}:
            width = getattr(operation, 'width', None)
            if not _number(tool.width_mm) or not _number(width) or tool.width_mm > width:
                reasons.append('groove_tool_too_wide')
        if role in {'thread_id', 'thread_od'}:
            pitch, angle = getattr(operation, 'pitch', None), getattr(operation, 'angle', None)
            limits = tool.pitch_range
            if (not isinstance(limits, (tuple, list)) or len(limits) != 2 or not all(_number(x) for x in limits)
                    or not _number(pitch) or not limits[0] <= pitch <= limits[1]):
                reasons.append('thread_pitch_not_supported')
            if not _number(tool.profile_angle) or not _number(angle) or abs(tool.profile_angle - angle) > 1e-6:
                reasons.append('thread_angle_not_supported')
        if explicit is not None and (not isinstance(explicit, TurningToolSpec) or explicit.to_dict() != tool.to_dict()):
            reasons.append('explicit_tool_does_not_match')
        if reasons:
            result.rejected_tools.append({'tool_id': tool.catalog_id, 'role': tool.role, 'reasons': reasons})
        elif result.selected_tool is None:
            result.selected_tool = deepcopy(tool)
    if result.selected_tool is None:
        result.errors.append(f'No owned Turning assembly supports the {role or name} operation.')
        descriptions = {'not_owned': 'The assembly is not available in the owned inventory.',
            'incomplete_insert_or_holder': 'The insert, holder, or tool hand is incomplete.',
            'invalid_insert_geometry': 'The insert nose radius or tip direction is invalid.',
            'minimum_bore_exceeded': 'The operation diameter is smaller than the recorded minimum bore.',
            'cutting_depth_exceeded': 'The cutting depth exceeds the recorded tool limit or the limit is missing.',
            'groove_tool_too_wide': 'The groove tool is wider than the operation or its width is missing.',
            'thread_pitch_not_supported': 'The thread pitch is outside the recorded tool range or the range is missing.',
            'thread_angle_not_supported': 'The thread angle does not match the recorded insert profile.',
            'explicit_tool_does_not_match': 'The explicit tool does not match the current inventory record.'}
        result.errors.extend(f"{row['tool_id']}: {descriptions[reason]}" for row in result.rejected_tools
            if row['role'] == role for reason in row['reasons'] if reason in descriptions)
        raise ToolUnavailableError(result)
    return result.selected_tool, result
