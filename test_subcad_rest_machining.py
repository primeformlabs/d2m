"""Rest machining of a profile pocket: a smaller cutter cuts only what a larger cutter left (D2O #748)."""
import math

import cadquery as cq
from subcad import Stock, ToolSpec, tool_inventory


def path_length(record):
    moves = record['toolpath']['moves']
    total = 0.0
    for first, second in zip(moves, moves[1:]):
        if second['type'] in {'feed', 'plunge'}:
            total += math.dist(first['position'], second['position'])
    return total


big = ToolSpec('flat_endmill', 16, catalog_id='EM16', physical_assembly={'owned': True})
small = ToolSpec('flat_endmill', 4, catalog_id='EM4', physical_assembly={'owned': True})
pocket = {'type': 'rectangle', 'length': 44, 'width': 24}

with tool_inventory([big, small], revision='rest-test', strict=True):
    stock = Stock.rectangular(60, 40, 20)
    top = stock._shape.val().BoundingBox().zmax
    corner = cq.Vector(-21.2, -11.2, top - 3)  # 0.8 mm from both walls of the pocket corner, 3 mm below the top
    roughed = stock.profile_pocket(pocket, 6, finish_allowance=0)
    assert roughed.process_plan()['operations'][-1]['tool']['diameter_mm'] == 16
    assert roughed._shape.val().isInside(corner), 'the 16 mm cutter leaves material in the corner'

    rested = roughed.profile_pocket(pocket, 6, finish_allowance=0, max_tool_diameter=4, rest_from_diameter=16)
    rest = rested.process_plan()['operations'][-1]
    assert rest['tool']['diameter_mm'] == 4
    assert not rested._shape.val().isInside(corner), 'the rest pass cuts the corner'
    # The corner keeps the 4 mm cutter radius: a point 0.3 mm from both walls stays material.
    assert rested._shape.val().isInside(cq.Vector(-21.7, -11.7, top - 3))

    full = stock.profile_pocket(pocket, 6, finish_allowance=0, max_tool_diameter=4).process_plan()['operations'][-1]
    assert full['tool']['diameter_mm'] == 4
    assert path_length(rest) < 0.35 * path_length(full), (path_length(rest), path_length(full))

    try:
        roughed.profile_pocket(pocket, 6, finish_allowance=0, tool=big, rest_from_diameter=4)
    except ValueError as error:
        assert 'smaller than the previous cutter' in str(error)
    else:
        raise AssertionError('a rest pass with a larger cutter must be refused')

print('rest machining ok', round(path_length(rest), 1), 'mm rest path;', round(path_length(full), 1), 'mm full 4 mm clear')
