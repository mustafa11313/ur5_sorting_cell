import math

from sorting_cell_task.grasp import (PartFootprint, candidate_angles, finger_points,
                                     footprint_distance, plan_grasp, wrap_half)


def test_cube_has_two_distinct_grasp_angles():
    angles = candidate_angles(PartFootprint(0, 0, 0.3, 0.04, 0.04, 0.04, 'cube'))
    assert len(angles) == 2
    assert math.isclose(abs(wrap_half(angles[1] - angles[0])), math.pi / 2, abs_tol=1e-6)


def test_hex_nut_has_three_grasp_angles():
    assert len(candidate_angles(PartFootprint(0, 0, 0.0, 0.058, 0.05, 0.025, 'hex_nut'))) == 3


def test_isolated_part_prefers_wrist_friendly_angle_and_safe_height():
    part = PartFootprint(0.1, 0.6, 0.2, 0.04, 0.04, 0.04, 'cube')
    g = plan_grasp(part, belt_z=0.72, reference_psi=0.0)
    assert g is not None
    assert math.isclose(g.psi, 0.2, abs_tol=1e-6)        # closest of {0.2, 0.2 - pi/2}
    assert math.isclose(g.z, 0.74, abs_tol=1e-6)         # part centre
    assert g.close_to < g.opening <= 0.08


def test_flat_part_keeps_fingertips_above_belt():
    part = PartFootprint(0, 0.6, 0.0, 0.058, 0.05, 0.025, 'hex_nut')
    g = plan_grasp(part, belt_z=0.72)
    assert g.z - 0.015 >= 0.72 + 0.004 - 1e-9


def test_touching_neighbour_forces_other_grasp_axis():
    part = PartFootprint(0.0, 0.6, 0.0, 0.04, 0.04, 0.04, 'cube')
    # neighbour touching the part on its -y side: fingers closing along y would hit it
    neighbour = PartFootprint(0.0, 0.6 - 0.042, 0.0, 0.04, 0.04, 0.04, 'cube')
    g = plan_grasp(part, [neighbour], reference_psi=0.0)
    assert g is not None
    # tool y (closing axis) must now be along world x, i.e. psi = +-pi/2
    assert math.isclose(abs(g.psi), math.pi / 2, abs_tol=1e-6)
    assert footprint_distance(finger_points(part, g.psi, g.opening), neighbour) > 0


def test_boxed_in_part_is_rejected():
    part = PartFootprint(0.0, 0.6, 0.0, 0.04, 0.04, 0.04, 'cube')
    ring = [PartFootprint(0.0 + dx, 0.6 + dy, 0.0, 0.04, 0.04, 0.04, 'cube')
            for dx, dy in ((0.041, 0), (-0.041, 0), (0, 0.041), (0, -0.041))]
    assert plan_grasp(part, ring) is None
