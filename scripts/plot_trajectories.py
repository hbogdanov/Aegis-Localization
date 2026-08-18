#!/usr/bin/env python3
"""Plot estimated and ground-truth trajectories and position error over time."""
import argparse
import os
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_ROOT = REPO_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from aegis_eval import plot_position_error, plot_trajectory_overlay


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--est', required=True)
    parser.add_argument('--gt', required=True)
    parser.add_argument('--out', required=True, help='Output path for trajectory plot')
    parser.add_argument('--out-error', default=None, help='Output path for position error plot')
    args = parser.parse_args()

    plot_trajectory_overlay(args.est, args.gt, args.out)
    print(f'Wrote trajectory plot to {args.out}')

    if args.out_error:
        out_err = args.out_error
    else:
        est_stem = os.path.splitext(os.path.basename(args.est))[0]
        out_err = os.path.join(os.path.dirname(args.out), f'{est_stem}_position_error.png')
    plot_position_error(args.est, args.gt, out_err)
    print(f'Wrote position error plot to {out_err}')


if __name__ == '__main__':
    main()
