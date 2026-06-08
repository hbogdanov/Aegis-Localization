#!/usr/bin/env python3
"""Plot estimated and ground-truth trajectories and position error over time."""
import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load_csv(path):
    df = pd.read_csv(path)
    # coerce timestamp to numeric and drop invalid rows
    df['timestamp'] = pd.to_numeric(df.get('timestamp'), errors='coerce')
    df = df.dropna(subset=['timestamp'])
    df = df.sort_values('timestamp').reset_index(drop=True)
    return df


def ensure_dir(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)


def wrap_angle(diff):
    return np.arctan2(np.sin(diff), np.cos(diff))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--est', required=True)
    parser.add_argument('--gt', required=True)
    parser.add_argument('--out', required=True, help='Output path for trajectory plot')
    parser.add_argument('--out-error', default=None, help='Output path for position error plot')
    args = parser.parse_args()

    est = load_csv(args.est)
    gt = load_csv(args.gt)

    # Trajectory plot
    ensure_dir(args.out)
    plt.figure(figsize=(8, 8))
    plt.plot(gt['x'], gt['y'], label='ground truth', linewidth=2)
    plt.plot(est['x'], est['y'], label='estimate', linewidth=2)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Trajectory: estimate vs ground truth')
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(args.out)
    print(f'Wrote trajectory plot to {args.out}')
    plt.close()

    # Position error over time: align by nearest timestamp
    merged = pd.merge_asof(est.sort_values('timestamp'), gt.sort_values('timestamp'), on='timestamp', suffixes=('_est', '_gt'), direction='nearest')
    dx = merged['x_est'].to_numpy() - merged['x_gt'].to_numpy()
    dy = merged['y_est'].to_numpy() - merged['y_gt'].to_numpy()
    pos_err = np.sqrt(dx * dx + dy * dy)

    out_err = args.out_error or os.path.join(os.path.dirname(args.out), 'ekf_position_error.png')
    ensure_dir(out_err)
    plt.figure(figsize=(8, 3))
    plt.plot(merged['timestamp'], pos_err, label='position error (m)')
    plt.xlabel('timestamp')
    plt.ylabel('position error (m)')
    plt.title('Position error over time')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(out_err)
    print(f'Wrote position error plot to {out_err}')


if __name__ == '__main__':
    main()
