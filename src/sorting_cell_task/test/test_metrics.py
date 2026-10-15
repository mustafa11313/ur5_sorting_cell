from sorting_cell_task.cell import bin_for_class, expected_bin_for_part
from sorting_cell_task.metrics_node import report_markdown, summarize


def event(pred, attached, result='sorted', attempts=1, cycle_time=10.0, reason=''):
    return {'predicted_class': pred, 'attached_part': attached, 'result': result,
            'attempts': attempts, 'cycle_time': cycle_time, 'failure_reason': reason}


def test_sorting_rules():
    assert bin_for_class('cube', 0.9) == 'bin_cube'
    assert bin_for_class('hex_nut_defect', 0.9) == 'bin_reject'
    assert bin_for_class('cylinder', 0.2) == 'bin_reject'
    assert expected_bin_for_part('part_0003__hex_nut__ok') == 'bin_hex_nut'
    assert expected_bin_for_part('part_0004__cube__defect') == 'bin_reject'


def test_summary():
    state = {
        'spawned': {f'p{i}' for i in range(6)},
        'binned': {'part_0000__cube__ok': 'bin_cube',
                   'part_0001__cube__defect': 'bin_cube',       # wrong bin
                   'part_0002__cylinder__ok': 'bin_cylinder'},
        'missed': {'part_0005__cube__ok'},
        'dropped': set(),
        'sort_events': [
            event('cube', 'part_0000__cube__ok', cycle_time=8.0),
            event('cube', 'part_0001__cube__defect', cycle_time=10.0),
            event('cylinder', 'part_0002__cylinder__ok', attempts=2, cycle_time=12.0),
            event('hex_nut', '', result='grasp_failed', attempts=2, reason='nothing in the gripper'),
        ],
    }
    m = summarize(state, elapsed_s=120.0, scenario='unit')
    assert m['parts_binned'] == 3
    assert m['parts_correct_bin'] == 2
    assert m['sorting_accuracy'] == round(2 / 3, 4)
    assert m['pick_attempts'] == 6 and m['successful_grasps'] == 3
    assert m['pick_success_rate'] == 0.5
    assert m['throughput_ppm'] == 1.0
    assert m['cycle_time_median_s'] == 10.0
    assert m['defect_recall'] == 0.0            # the defective cube was missed
    assert m['per_class']['cube']['precision'] == 0.5
    assert m['failure_reasons'] == {'nothing in the gripper': 1}
    assert '| Sorting accuracy | 66.7 % |' in report_markdown(m)
