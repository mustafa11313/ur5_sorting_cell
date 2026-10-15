"""Evaluation of the sorting cell from ground truth (Gazebo) and task events.

Inputs
  /parts/events   std_msgs/String   from the Gazebo plugin (ground truth):
                  "spawned <part>", "binned <part> <bin>", "missed <part>", "dropped <part>"
  /sorting/events SortEvent         one per pick-and-place attempt from the task node

Metrics
  pick success rate      successful grasps / grasp attempts
  sorting accuracy       parts in the correct bin / parts in any bin
  throughput             correctly sorted parts per minute (sim time)
  cycle time             target selected -> part released (mean / median / p95)
  classification         confusion matrix, per-class precision and recall
                         (prediction vs. the class of the part actually grasped)
  failures               failed grasps, planning/execution failures, missed and dropped parts

Results are written to <results_dir>/metrics.json, report.md and events.csv,
periodically and on shutdown. With duration > 0 the node exits after that many
simulated seconds, which ends an evaluation launch.
"""
import csv
import json
import os
import statistics
from collections import Counter, defaultdict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sorting_cell_interfaces.msg import SortEvent

from .cell import expected_bin_for_part

CLASSES = ['cube', 'cube_defect', 'cylinder', 'cylinder_defect', 'hex_nut', 'hex_nut_defect']
RESULT_NAMES = {0: 'sorted', 1: 'grasp_failed', 2: 'planning_failed', 3: 'lost'}


def gt_class(part_name):
    try:
        _, ptype, quality = part_name.split('__')
    except ValueError:
        return None
    return ptype + ('_defect' if quality == 'defect' else '')


def precision_recall(confusion, classes):
    out = {}
    for c in classes:
        tp = confusion[c][c]
        fp = sum(confusion[o][c] for o in classes if o != c)
        fn = sum(confusion[c][o] for o in confusion[c] if o != c)
        out[c] = {
            'precision': tp / (tp + fp) if tp + fp else None,
            'recall': tp / (tp + fn) if tp + fn else None,
            'support': tp + fn,
        }
    return out


def summarize(state, elapsed_s, scenario=''):
    """Compute the metrics dict from the raw counters (pure function, unit tested)."""
    binned = state['binned']
    correct = sum(1 for part, b in binned.items() if expected_bin_for_part(part) == b)
    events = state['sort_events']
    attempts = sum(e['attempts'] for e in events)
    grasps_ok = sum(1 for e in events if e['attached_part'])
    sorted_ev = [e for e in events if e['result'] == 'sorted']
    cycle_times = sorted(e['cycle_time'] for e in sorted_ev)

    confusion = defaultdict(Counter)
    for e in events:
        if e['attached_part'] and e['predicted_class']:
            confusion[gt_class(e['attached_part'])][e['predicted_class']] += 1
    pr = precision_recall(confusion, CLASSES)
    n_cls = sum(sum(c.values()) for c in confusion.values())
    cls_correct = sum(confusion[c][c] for c in CLASSES)
    # binary defect detection (defect = positive)
    tp = sum(v for g, row in confusion.items() for p, v in row.items()
             if g and g.endswith('_defect') and p.endswith('_defect'))
    fp = sum(v for g, row in confusion.items() for p, v in row.items()
             if g and not g.endswith('_defect') and p.endswith('_defect'))
    fn = sum(v for g, row in confusion.items() for p, v in row.items()
             if g and g.endswith('_defect') and not p.endswith('_defect'))
    minutes = max(elapsed_s / 60.0, 1e-9)

    def ratio(a, b):
        return round(a / b, 4) if b else None

    return {
        'scenario': scenario,
        'sim_time_s': round(elapsed_s, 1),
        'parts_spawned': len(state['spawned']),
        'parts_binned': len(binned),
        'parts_correct_bin': correct,
        'parts_missed': len(state['missed']),
        'parts_dropped': len(state['dropped']),
        'pick_attempts': attempts,
        'successful_grasps': grasps_ok,
        'failed_grasps': attempts - grasps_ok,
        'planning_failures': sum(1 for e in events if e['result'] == 'planning_failed'),
        'pick_success_rate': ratio(grasps_ok, attempts),
        'sorting_accuracy': ratio(correct, len(binned)),
        'throughput_ppm': round(correct / minutes, 2),
        'cycle_time_mean_s': round(statistics.mean(cycle_times), 2) if cycle_times else None,
        'cycle_time_median_s': round(statistics.median(cycle_times), 2) if cycle_times else None,
        'cycle_time_p95_s': round(cycle_times[int(0.95 * (len(cycle_times) - 1))], 2)
        if cycle_times else None,
        'classification_accuracy': ratio(cls_correct, n_cls),
        'defect_precision': ratio(tp, tp + fp),
        'defect_recall': ratio(tp, tp + fn),
        'per_class': pr,
        'confusion': {g: dict(row) for g, row in confusion.items() if g},
        'failure_reasons': dict(Counter(e['failure_reason'] for e in events
                                        if e['failure_reason'])),
    }


def report_markdown(m):
    def fmt(v, pct=False):
        if v is None:
            return 'n/a'
        return f'{100 * v:.1f} %' if pct else f'{v}'

    lines = [
        f"# Sorting cell evaluation: {m['scenario'] or 'run'}",
        '',
        f"Simulated time: {m['sim_time_s']} s, parts spawned: {m['parts_spawned']}",
        '',
        '| Metric | Value |',
        '|---|---|',
        f"| Pick success rate | {fmt(m['pick_success_rate'], True)} |",
        f"| Sorting accuracy | {fmt(m['sorting_accuracy'], True)} |",
        f"| Throughput (correct parts / min) | {m['throughput_ppm']} |",
        f"| Cycle time mean / median / p95 [s] | {fmt(m['cycle_time_mean_s'])} / "
        f"{fmt(m['cycle_time_median_s'])} / {fmt(m['cycle_time_p95_s'])} |",
        f"| Classification accuracy | {fmt(m['classification_accuracy'], True)} |",
        f"| Defect precision / recall | {fmt(m['defect_precision'], True)} / "
        f"{fmt(m['defect_recall'], True)} |",
        f"| Failed grasps | {m['failed_grasps']} of {m['pick_attempts']} attempts |",
        f"| Planning / execution failures | {m['planning_failures']} |",
        f"| Missed (left the belt) / dropped | {m['parts_missed']} / {m['parts_dropped']} |",
        '',
        '| Class | Precision | Recall | Support |',
        '|---|---|---|---|',
    ]
    for c, v in m['per_class'].items():
        lines.append(f"| {c} | {fmt(v['precision'], True)} | {fmt(v['recall'], True)} | "
                     f"{v['support']} |")
    if m['failure_reasons']:
        lines += ['', 'Failure reasons: ' + ', '.join(
            f'{k} ({v})' for k, v in m['failure_reasons'].items())]
    return '\n'.join(lines) + '\n'


class MetricsNode(Node):
    def __init__(self):
        super().__init__('sorting_metrics')
        self.declare_parameter('results_dir', os.path.expanduser('~/sorting_results'))
        self.declare_parameter('scenario', '')
        self.declare_parameter('duration', 0.0)      # sim seconds, 0 = run forever
        self.declare_parameter('warmup', 0.0)        # ignore parts spawned before this
        self.results_dir = self.get_parameter('results_dir').value
        self.scenario = self.get_parameter('scenario').value
        self.duration = self.get_parameter('duration').value
        os.makedirs(self.results_dir, exist_ok=True)
        self.state = {'spawned': set(), 'binned': {}, 'missed': set(), 'dropped': set(),
                      'sort_events': []}
        self.start = None
        self.done = False
        self.create_subscription(String, '/parts/events', self._on_part_event, 50)
        self.create_subscription(SortEvent, '/sorting/events', self._on_sort_event, 50)
        self.create_timer(10.0, self.write)
        self.create_timer(1.0, self._check_duration)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_part_event(self, msg):
        if self.start is None:
            self.start = self._now()
        parts = msg.data.split()
        if len(parts) < 2:
            return
        kind, name = parts[0], parts[1]
        if kind == 'spawned':
            self.state['spawned'].add(name)
        elif kind == 'binned' and len(parts) >= 3:
            self.state['binned'][name] = parts[2]
        elif kind == 'missed':
            self.state['missed'].add(name)
        elif kind == 'dropped':
            self.state['dropped'].add(name)

    def _on_sort_event(self, ev):
        self.state['sort_events'].append({
            't': ev.stamp.sec + ev.stamp.nanosec * 1e-9,
            'cycle': ev.cycle, 'part_id': ev.part_id, 'predicted_class': ev.predicted_class,
            'confidence': round(ev.confidence, 3), 'target_bin': ev.target_bin,
            'attached_part': ev.attached_part, 'result': RESULT_NAMES.get(ev.result, '?'),
            'failure_reason': ev.failure_reason, 'attempts': ev.attempts,
            'cycle_time': round(ev.cycle_time, 3),
        })

    def _check_duration(self):
        if self.duration > 0 and self.start is not None and \
                self._now() - self.start >= self.duration and not self.done:
            self.write()
            self.get_logger().info(f'evaluation finished, results in {self.results_dir}')
            self.done = True

    def write(self):
        elapsed = 0.0 if self.start is None else self._now() - self.start
        m = summarize(self.state, elapsed, self.scenario)
        with open(os.path.join(self.results_dir, 'metrics.json'), 'w') as f:
            json.dump(m, f, indent=2)
        with open(os.path.join(self.results_dir, 'report.md'), 'w') as f:
            f.write(report_markdown(m))
        events = self.state['sort_events']
        if events:
            with open(os.path.join(self.results_dir, 'events.csv'), 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(events[0]))
                w.writeheader()
                w.writerows(events)
        self.get_logger().info(
            f"t={m['sim_time_s']}s sorted={m['parts_correct_bin']}/{m['parts_binned']} "
            f"pick={m['pick_success_rate']} acc={m['sorting_accuracy']} "
            f"ppm={m['throughput_ppm']} missed={m['parts_missed']}")


def main(args=None):
    rclpy.init(args=args)
    node = MetricsNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.write()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()
