#!/usr/bin/env python3
"""Evaluate estimated trajectory against ground truth.

Usage:
  python3 scripts/evaluate_trajectory.py --est results/metrics/ekf.csv --gt results/metrics/ground_truth.csv
"""
import argparse
import json
import os
import sys

import numpy as np
import pandas as pd


def wrap_angle(diff):
    return np.arctan2(np.sin(diff), np.cos(diff))


def load_csv(path):
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    df = pd.read_csv(path)
    if 'timestamp' not in df.columns:
        raise ValueError(f"CSV missing 'timestamp' column: {path}")
    required = ['x', 'y', 'yaw']
    for c in required:
        if c not in df.columns:
            raise ValueError(f"CSV missing '{c}' column: {path}")
    df = df[['timestamp', 'x', 'y', 'yaw']].copy()
    df['timestamp'] = pd.to_numeric(df['timestamp'], errors='coerce')
    df = df.dropna(subset=['timestamp'])
    df = df.sort_values('timestamp').reset_index(drop=True)
    return df


def align_by_timestamp(est, gt):
    est = est.sort_values('timestamp').reset_index(drop=True)
    gt = gt.sort_values('timestamp').reset_index(drop=True)
    merged = pd.merge_asof(est, gt, on='timestamp', suffixes=('_est', '_gt'), direction='nearest')
    return merged


def compute_metrics(merged):
    dx = merged['x_est'].to_numpy() - merged['x_gt'].to_numpy()
    dy = merged['y_est'].to_numpy() - merged['y_gt'].to_numpy()
    pos_err = np.sqrt(dx * dx + dy * dy)
    ate_rmse = float(np.sqrt(np.mean(pos_err ** 2))) if pos_err.size > 0 else float('nan')
    final_drift = float(pos_err[-1]) if pos_err.size > 0 else float('nan')

    yaw_diff = wrap_angle(merged['yaw_est'].to_numpy() - merged['yaw_gt'].to_numpy())
    yaw_rmse = float(np.sqrt(np.mean(yaw_diff ** 2))) if yaw_diff.size > 0 else float('nan')

    return {
        'num_samples': int(len(merged)),
        'ate_rmse': ate_rmse,
        'final_drift': final_drift,
        'yaw_rmse': yaw_rmse,
    }


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
    parser.add_argument('--out-json', default='results/metrics/ekf_metrics.json', help='Optional output JSON')
    args = parser.parse_args()

    try:
        est = load_csv(args.est)
        gt = load_csv(args.gt)
    except Exception as e:
        print(f'Error loading CSVs: {e}', file=sys.stderr)
        sys.exit(2)

    merged = align_by_timestamp(est, gt)
    metrics = compute_metrics(merged)
    print_table(metrics)

    try:
        os.makedirs(os.path.dirname(args.out_json), exist_ok=True)
        with open(args.out_json, 'w') as f:
            json.dump(metrics, f, indent=2)
        print(f'Wrote metrics to {args.out_json}')
    except Exception:
        pass


if __name__ == '__main__':
    main()
