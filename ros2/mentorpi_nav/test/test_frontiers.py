"""Frontier search on a synthetic floor plan: two rooms joined by a door, half explored."""
import math

import numpy as np

from mentorpi_nav.frontiers import Grid, find_frontiers, frontier_mask, masks


def plan():
    d = np.full((80, 120), -1, np.int8)          # 4 m x 6 m at 5 cm
    d[5:75, 5:60] = 0                            # left room seen
    d[5:75, 5] = d[5:75, 59] = 100               # walls
    d[5, 5:60] = d[74, 5:60] = 100
    d[35:52, 59] = 0                             # 0.85 m door, unknown beyond
    return Grid(d, 0.05, 0.0, 0.0)


def test_frontier_is_the_door():
    g = plan()
    fr, _ = find_frontiers(g, (1.0, 2.0))
    assert len(fr) == 1
    f = fr[0]
    assert abs(f.cy - 2.15) < 0.1 and abs(f.cx - 2.98) < 0.1      # door centre
    assert f.x < 2.98 and abs(f.y - 2.15) < 0.15                     # goal on the robot's side
    assert 1.5 < f.distance_m < 2.5


def test_narrow_gap_is_not_reachable():
    g = plan()
    g.data[35:52, 59] = 100
    g.data[42:46, 59] = 0                        # 0.2 m gap: frontier, but the robot can't pass
    fr, _ = find_frontiers(g, (1.0, 2.0), robot_radius=0.15, approach_m=0.1)
    assert fr == []


def test_blacklist():
    g = plan()
    fr, _ = find_frontiers(g, (1.0, 2.0))
    fr2, _ = find_frontiers(g, (1.0, 2.0), blacklist=[(fr[0].x, fr[0].y)])
    assert fr2 == []


def test_frontier_mask_only_free_cells():
    g = plan()
    free, occ, unk = masks(g)
    f = frontier_mask(free, unk)
    assert not (f & ~free).any()
