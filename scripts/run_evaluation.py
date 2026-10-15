#!/usr/bin/env python3
"""Run the benchmark scenarios headless and aggregate the metrics into one table.

    python3 scripts/run_evaluation.py --duration 600 --backend yolo
    python3 scripts/run_evaluation.py --scenarios nominal touching --duration 300 --backend classical

Each scenario is a full launch of the cell (Gazebo server only, no RViz) that
shuts itself down after --duration simulated seconds. Results go to
results/<backend>/<scenario>/ and the summary to results/<backend>/summary.md.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

SCENARIOS = ['nominal', 'touching', 'new_variants', 'dim_light', 'fast_belt']

COLUMNS = [
    ('Scenario', 'scenario', None),
    ('Parts', 'parts_spawned', None),
    ('Pick success', 'pick_success_rate', 'pct'),
    ('Sorting acc.', 'sorting_accuracy', 'pct'),
    ('Throughput [ppm]', 'throughput_ppm', None),
    ('Cycle time [s]', 'cycle_time_median_s', None),
    ('Class. acc.', 'classification_accuracy', 'pct'),
    ('Defect P / R', ('defect_precision', 'defect_recall'), 'pct'),
    ('Failed grasps', 'failed_grasps', None),
    ('Plan fails', 'planning_failures', None),
    ('Missed', 'parts_missed', None),
]


def run_isolated(cmd, log, timeout):
    """Run a launch in its own process group and make sure nothing outlives it.

    `ign gazebo` is started through a ruby wrapper; when the launch shuts down,
    the wrapper is killed but the server process can survive and keep
    publishing /clock and /controller_manager, which breaks the next scenario.
    """
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    subprocess.run(['pkill', '-9', '-f', 'ign gazebo -s'], check=False)
    time.sleep(3)


def fmt(v, kind):
    if v is None:
        return 'n/a'
    return f'{100 * v:.1f} %' if kind == 'pct' else str(v)


def summary_table(rows):
    lines = ['| ' + ' | '.join(c[0] for c in COLUMNS) + ' |',
             '|' + '---|' * len(COLUMNS)]
    for m in rows:
        cells = []
        for _, key, kind in COLUMNS:
            if isinstance(key, tuple):
                cells.append(' / '.join(fmt(m.get(k), kind) for k in key))
            else:
                cells.append(fmt(m.get(key), kind))
        lines.append('| ' + ' | '.join(cells) + ' |')
    return '\n'.join(lines) + '\n'


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--scenarios', nargs='+', default=SCENARIOS)
    p.add_argument('--duration', type=float, default=600.0, help='simulated seconds per scenario')
    p.add_argument('--backend', default='yolo', choices=['yolo', 'classical'])
    p.add_argument('--results', default='results')
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--skip-run', action='store_true', help='only rebuild the summary table')
    args = p.parse_args()

    out_root = os.path.abspath(os.path.join(args.results, args.backend))
    rows = []
    for scenario in args.scenarios:
        out = os.path.join(out_root, scenario)
        if not args.skip_run:
            print(f'=== {scenario}: {args.duration:.0f} s simulated ({args.backend}) ===', flush=True)
            os.makedirs(out, exist_ok=True)
            cmd = ['ros2', 'launch', 'sorting_cell_bringup', 'sorting_cell.launch.py',
                   f'scenario:={scenario}', 'headless:=true', 'rviz:=false',
                   f'backend:={args.backend}', f'seed:={args.seed}',
                   f'eval_duration:={args.duration}', f'results_dir:={out}']
            with open(os.path.join(out, 'launch.log'), 'w') as log:
                run_isolated(cmd, log, timeout=args.duration * 4 + 300)
        path = os.path.join(out, 'metrics.json')
        if os.path.exists(path):
            with open(path) as f:
                rows.append(json.load(f))
        else:
            print(f'no metrics for {scenario}', file=sys.stderr)

    table = summary_table(rows)
    with open(os.path.join(out_root, 'summary.md'), 'w') as f:
        f.write(f'# Evaluation ({args.backend} backend, {args.duration:.0f} s per scenario, '
                f'seed {args.seed})\n\n' + table)
    print(table)


if __name__ == '__main__':
    main()
