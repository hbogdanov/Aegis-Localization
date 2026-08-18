#!/usr/bin/env python3
"""Evaluate estimated trajectory against ground truth."""
import argparse
import os
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_ROOT = REPO_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from aegis_eval import align_by_timestamp, compute_metrics, load_trajectory_csv, write_json


def load_csv(path):
    return load_trajectory_csv(path)


def print_table(metrics):
    print('\nTrajectory evaluation results')
    print('-----------------------------')
    print(f"Samples:       {metrics['num_samples']}")
    print(f"ATE RMSE (m):  {metrics['ate_rmse']:.4f}")
    print(f"Final drift (m): {metrics['final_drift']:.4f}")
    print(f"Yaw RMSE (rad): {metrics['yaw_rmse']:.4f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--est', required=True, help='Estimated trajectory CSV')
    parser.add_argument('--gt', required=True, help='Ground-truth trajectory CSV')
    parser.add_argument('--out-json', default=None, help='Optional output JSON')
    args = parser.parse_args()

    try:
        est = load_trajectory_csv(args.est)
        gt = load_trajectory_csv(args.gt)
    except Exception as e:
        print(f'Error loading CSVs: {e}', file=sys.stderr)
        sys.exit(2)

    try:
        merged = align_by_timestamp(est, gt)
        metrics = compute_metrics(merged)
    except Exception as e:
        print(f'Error computing metrics: {e}', file=sys.stderr)
        sys.exit(2)

    print_table(metrics)

    try:
        out_json = args.out_json
        if out_json is None:
            est_stem = os.path.splitext(os.path.basename(args.est))[0]
            out_json = os.path.join('results', 'metrics', f'{est_stem}_metrics.json')
        write_json(out_json, metrics)
        print(f'Wrote metrics to {out_json}')
    except Exception:
        pass


if __name__ == '__main__':
    main()
