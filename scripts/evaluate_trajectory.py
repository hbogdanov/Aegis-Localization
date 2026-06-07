#!/usr/bin/env python3

import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Evaluate trajectory error metrics.')
    parser.add_argument('--est', required=True)
    parser.add_argument('--gt', required=True)
    args = parser.parse_args()
    print(f'Evaluating trajectories: estimated={args.est}, ground truth={args.gt}')
