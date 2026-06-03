#!/usr/bin/env python3
"""
train.py
=========
Training script offline untuk Visual Regression Model AMR.

Usage:
    python3 train.py --dataset /home/azhar/datasets/run_20260501_140000 \\
                     --output  /home/azhar/models \\
                     --num-regions 9

Algoritma: Random Forest Regressor (multi-output: steering, velocity).
Output: vr_model.pkl, vr_scaler.pkl, train_report.txt, scatter_plot.png

Tip: jalankan setelah collect dataset (lihat data_collector_node).
"""
import argparse
import os
import sys
from glob import glob

import numpy as np
import pandas as pd

import joblib
import matplotlib
matplotlib.use('Agg')   # no display
import matplotlib.pyplot as plt

from sklearn.ensemble import RandomForestRegressor
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.multioutput import MultiOutputRegressor
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score


# Penting: kita IMPORT modul feature_extractor langsung (bukan dari ament_index),
# supaya script ini bisa jalan tanpa workspace di-source. Cari path-nya:
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.join(os.path.dirname(THIS_DIR), 'amr_visual_regression')
if SRC_DIR not in sys.path:
    sys.path.insert(0, os.path.dirname(THIS_DIR))

from amr_visual_regression.feature_extractor import extract_features, feature_names


def load_dataset(dataset_dir: str):
    """Load semua frame dari folder run_*. Bisa terima 1 folder atau parent."""
    csv_path = os.path.join(dataset_dir, 'labels.csv')
    if not os.path.exists(csv_path):
        # Mungkin parent folder yang berisi banyak run_*
        sub_runs = sorted(glob(os.path.join(dataset_dir, 'run_*')))
        if sub_runs:
            print(f'[INFO] Found {len(sub_runs)} run folders, loading all...')
            all_X, all_y_steer, all_y_vel = [], [], []
            for run in sub_runs:
                X, ys, yv = load_dataset(run)
                if X is not None:
                    all_X.append(X)
                    all_y_steer.append(ys)
                    all_y_vel.append(yv)
            if not all_X:
                return None, None, None
            return (np.vstack(all_X),
                    np.concatenate(all_y_steer),
                    np.concatenate(all_y_vel))
        else:
            print(f'[ERROR] No labels.csv in {dataset_dir}')
            return None, None, None

    df = pd.read_csv(csv_path)
    print(f'[INFO] Loading {len(df)} frames from {dataset_dir}')

    X_list = []
    y_steer = []
    y_vel = []

    for idx, row in df.iterrows():
        depth_path = os.path.join(dataset_dir, row['depth_filename'])
        try:
            depth = np.load(depth_path)
        except Exception as e:
            print(f'[WARN] Skip {depth_path}: {e}')
            continue

        feats = extract_features(depth.astype(np.uint16))
        X_list.append(feats)
        y_steer.append(float(row['steering_cmd']))
        y_vel.append(float(row['velocity_cmd']))

        if (idx + 1) % 500 == 0:
            print(f'  ... {idx+1}/{len(df)} processed')

    return (np.array(X_list, dtype=np.float32),
            np.array(y_steer, dtype=np.float32),
            np.array(y_vel,   dtype=np.float32))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', required=True,
                        help='Folder run_<timestamp> atau folder parent yang berisi run_*')
    parser.add_argument('--output', default='./models',
                        help='Output folder untuk model dan scaler')
    parser.add_argument('--num-regions', type=int, default=9)
    parser.add_argument('--n-estimators', type=int, default=100)
    parser.add_argument('--max-depth', type=int, default=15)
    parser.add_argument('--test-size', type=float, default=0.15)
    parser.add_argument('--val-size',  type=float, default=0.15)
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    # ---- Load dataset ----
    X, y_steer, y_vel = load_dataset(args.dataset)
    if X is None:
        sys.exit(1)
    y = np.column_stack([y_steer, y_vel])
    print(f'[INFO] Dataset shape: X={X.shape}, y={y.shape}')

    # ---- Train/val/test split (60/20/20 ish) ----
    X_temp, X_test, y_temp, y_test = train_test_split(
        X, y, test_size=args.test_size, random_state=42)
    val_relative = args.val_size / (1.0 - args.test_size)
    X_train, X_val, y_train, y_val = train_test_split(
        X_temp, y_temp, test_size=val_relative, random_state=42)
    print(f'[INFO] Train={X_train.shape[0]}, '
          f'Val={X_val.shape[0]}, Test={X_test.shape[0]}')

    # ---- Scaler ----
    scaler = StandardScaler()
    X_train_s = scaler.fit_transform(X_train)
    X_val_s = scaler.transform(X_val)
    X_test_s = scaler.transform(X_test)

    # ---- Train Random Forest ----
    print(f'[INFO] Training RandomForest '
          f'(n_estimators={args.n_estimators}, max_depth={args.max_depth})...')
    base = RandomForestRegressor(
        n_estimators=args.n_estimators,
        max_depth=args.max_depth,
        n_jobs=-1,
        random_state=42,
    )
    model = MultiOutputRegressor(base)
    model.fit(X_train_s, y_train)

    # ---- Evaluate ----
    def eval_split(X_s, y_, name):
        pred = model.predict(X_s)
        mae = mean_absolute_error(y_, pred, multioutput='raw_values')
        rmse = np.sqrt(mean_squared_error(y_, pred, multioutput='raw_values'))
        r2 = r2_score(y_, pred, multioutput='raw_values')
        print(f'  [{name}] MAE  steer={mae[0]:.4f}, vel={mae[1]:.4f}')
        print(f'  [{name}] RMSE steer={rmse[0]:.4f}, vel={rmse[1]:.4f}')
        print(f'  [{name}] R2   steer={r2[0]:+.4f}, vel={r2[1]:+.4f}')
        return mae, rmse, r2, pred

    print('[INFO] Evaluating...')
    eval_split(X_train_s, y_train, 'TRAIN')
    eval_split(X_val_s, y_val, 'VAL')
    mae_t, rmse_t, r2_t, pred_test = eval_split(X_test_s, y_test, 'TEST')

    # ---- Save model + scaler ----
    model_path = os.path.join(args.output, 'vr_model.pkl')
    scaler_path = os.path.join(args.output, 'vr_scaler.pkl')
    joblib.dump(model, model_path)
    joblib.dump(scaler, scaler_path)
    print(f'[OK] Saved model: {model_path}')
    print(f'[OK] Saved scaler: {scaler_path}')

    # ---- Scatter plot prediction vs ground truth ----
    fig, axs = plt.subplots(1, 2, figsize=(12, 5))
    for i, label in enumerate(['steering', 'velocity']):
        axs[i].scatter(y_test[:, i], pred_test[:, i], alpha=0.3, s=5)
        lim = max(abs(y_test[:, i]).max(), abs(pred_test[:, i]).max()) * 1.1
        axs[i].plot([-lim, lim], [-lim, lim], 'r--', linewidth=1)
        axs[i].set_xlabel(f'Ground truth {label}')
        axs[i].set_ylabel(f'Predicted {label}')
        axs[i].set_title(f'{label.capitalize()} (R2={r2_t[i]:+.3f})')
        axs[i].grid(True, alpha=0.3)
    plt.tight_layout()
    plot_path = os.path.join(args.output, 'scatter_plot.png')
    plt.savefig(plot_path, dpi=120)
    print(f'[OK] Saved scatter plot: {plot_path}')

    # ---- Report ----
    report_path = os.path.join(args.output, 'train_report.txt')
    with open(report_path, 'w') as f:
        f.write('AMR Visual Regression - Training Report\n')
        f.write('=' * 50 + '\n')
        f.write(f'Dataset:       {args.dataset}\n')
        f.write(f'N samples:     {X.shape[0]}\n')
        f.write(f'N features:    {X.shape[1]}\n')
        f.write(f'N regions:     {args.num_regions}\n')
        f.write(f'n_estimators:  {args.n_estimators}\n')
        f.write(f'max_depth:     {args.max_depth}\n')
        f.write('\n[TEST METRICS]\n')
        f.write(f'  MAE  steering={mae_t[0]:.4f}  velocity={mae_t[1]:.4f}\n')
        f.write(f'  RMSE steering={rmse_t[0]:.4f}  velocity={rmse_t[1]:.4f}\n')
        f.write(f'  R2   steering={r2_t[0]:+.4f}  velocity={r2_t[1]:+.4f}\n')
    print(f'[OK] Saved report: {report_path}')

    # ---- Sanity check ----
    if r2_t[0] < 0.3 or r2_t[1] < 0.3:
        print('\n[WARN] R2 score < 0.3 -> dataset mungkin terlalu noisy '
              'atau tidak cukup variasi. Pertimbangkan re-collect.')


if __name__ == '__main__':
    main()
