#!/usr/bin/env python3
"""Statistics and plots from a benchmark CSV file.

Usage: analyze_benchmark.py RESULTS.csv
"""
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def describe(series, unit):
    return (f'mean {series.mean():.1f} {unit}, std {series.std():.1f}, '
            f'median {series.median():.1f}, p95 {series.quantile(0.95):.1f}, '
            f'max {series.max():.1f}')


def main():
    path = Path(sys.argv[1])
    df = pd.read_csv(path)
    placed = df[df['true_success']]

    print(f'Cycles:               {len(df)}')
    print(f'True success rate:    {len(placed)}/{len(df)} ({100 * len(placed) / len(df):.0f}%)')
    print(f'Reported success:     {df["reported_success"].sum()}/{len(df)}')
    print(f'Position error:       {describe(placed["true_error_mm"], "mm")}')
    print(f'Yaw error (absolute): {describe(placed["true_yaw_error_deg"].abs(), "deg")}')
    print(f'Cycle time:           {describe(df["duration_s"], "s")}')

    # How well does the robot's own camera measurement match the truth?
    gap = (placed['estimated_error_mm'] - placed['true_error_mm']).abs()
    print(f'Self-check vs truth:  {describe(gap, "mm")}')

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))

    axes[0].hist(placed['true_error_mm'], bins=15, color='#4C72B0')
    axes[0].set_title('Placement position error')
    axes[0].set_xlabel('error [mm]')
    axes[0].set_ylabel('cycles')

    axes[1].scatter(placed['true_error_x_mm'], placed['true_error_y_mm'], color='#4C72B0')
    axes[1].axhline(0, color='grey', linewidth=0.8)
    axes[1].axvline(0, color='grey', linewidth=0.8)
    axes[1].set_aspect('equal')
    axes[1].set_title('Placement error in the table plane')
    axes[1].set_xlabel('x error [mm]')
    axes[1].set_ylabel('y error [mm]')

    axes[2].scatter(placed['true_error_mm'], placed['estimated_error_mm'], color='#4C72B0')
    limit = max(placed['true_error_mm'].max(), placed['estimated_error_mm'].max()) * 1.1
    axes[2].plot([0, limit], [0, limit], color='grey', linestyle='--', linewidth=0.8)
    axes[2].set_title('Camera self-check vs ground truth')
    axes[2].set_xlabel('true error [mm]')
    axes[2].set_ylabel('error measured by the robot [mm]')

    fig.tight_layout()
    output = path.with_suffix('.png')
    fig.savefig(output, dpi=120)
    print(f'Plot saved to {output}')


if __name__ == '__main__':
    main()