#!/usr/bin/env python3
"""
Parking behavior cloning training script.

Usage:
    python3 ~/ros2_ws/parking_data/train_parking.py

Reads all parking_*.csv files, trains a steering-only MLP,
saves to ~/ros2_ws/trained_parking.pkl
"""

import os
import glob
import csv
import pickle
import numpy as np
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_absolute_error

TRAINING_DIR = os.path.expanduser('~/ros2_ws/parking_data')
MODEL_PATH   = os.path.expanduser('~/ros2_ws/trained_parking.pkl')

FEATURE_COLS = ['spot_visible', 'spot_cx', 'spot_cy', 'spot_area', 'sonar_cm', 'sonar_trend']
LABEL_COL    = 'steering'   # speed is rule-based, only learn steering

SONAR_MAX = 200.0
STEER_MAX = 90.0


def load_data():
    files = sorted(glob.glob(os.path.join(TRAINING_DIR, 'parking_*.csv')))
    if not files:
        raise FileNotFoundError(f'No parking CSV files in {TRAINING_DIR}')

    rows = []
    for f in files:
        with open(f) as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                rows.append(row)

    print(f'Loaded {len(rows)} samples from {len(files)} session(s):')
    for f in files:
        print(f'  {os.path.basename(f)}')
    return rows


def build_arrays(rows):
    X, y = [], []
    skipped = 0
    for row in rows:
        try:
            visible = float(row['spot_visible'])
            cx      = float(row['spot_cx'])
            cy      = float(row['spot_cy'])
            area    = float(row['spot_area'])
            sonar   = float(row['sonar_cm'])
            trend   = float(row['sonar_trend'])
            steer   = float(row['steering'])
        except (ValueError, KeyError):
            skipped += 1
            continue

        sonar_norm = min(sonar, SONAR_MAX) / SONAR_MAX
        trend_norm = (trend + 1) / 2.0
        steer_norm = steer / STEER_MAX

        X.append([visible, cx, cy, area, sonar_norm, trend_norm])
        y.append(steer_norm)

    if skipped:
        print(f'Skipped {skipped} malformed rows.')

    return np.array(X, dtype=np.float32), np.array(y, dtype=np.float32)


def print_summary(X, y):
    steers = y * STEER_MAX
    visible_pct = X[:, 0].mean() * 100
    print(f'\nDataset summary ({len(X)} samples):')
    print(f'  Spot visible: {visible_pct:.0f}% of frames')
    print(f'  Steering — mean: {steers.mean():.1f}°  std: {steers.std():.1f}°  '
          f'min: {steers.min():.1f}°  max: {steers.max():.1f}°')
    left     = (steers < -5).sum()
    straight = (np.abs(steers) <= 5).sum()
    right    = (steers > 5).sum()
    print(f'  Left: {left} ({100*left/len(X):.0f}%)  '
          f'Straight: {straight} ({100*straight/len(X):.0f}%)  '
          f'Right: {right} ({100*right/len(X):.0f}%)')

    if visible_pct < 40:
        print('\n  WARNING: spot visible in less than 40% of frames.')
        print('  Consider doing more demos where the spot is in view the whole approach.')


def train(X, y):
    X_train, X_val, y_train, y_val = train_test_split(X, y, test_size=0.2, random_state=42)

    scaler = StandardScaler()
    X_train_s = scaler.fit_transform(X_train)
    X_val_s   = scaler.transform(X_val)

    print(f'\nTraining on {len(X_train)} samples, validating on {len(X_val)}...')

    model = MLPRegressor(
        hidden_layer_sizes=(64, 64, 32),
        activation='relu',
        solver='adam',
        learning_rate_init=0.001,
        max_iter=500,
        random_state=42,
        early_stopping=True,
        validation_fraction=0.1,
        n_iter_no_change=20,
    )
    model.fit(X_train_s, y_train)

    preds = model.predict(X_val_s)
    mae = mean_absolute_error(y_val, preds) * STEER_MAX
    print(f'Validation MAE — steering: {mae:.1f}° (out of 90°)')
    print(f'Stopped after {model.n_iter_} iterations.')
    return model, scaler


def save_model(model, scaler):
    payload = {'model': model, 'scaler': scaler}
    with open(MODEL_PATH, 'wb') as f:
        pickle.dump(payload, f)
    print(f'\nModel saved → {MODEL_PATH}')


if __name__ == '__main__':
    print('=== Parking Behavior Cloning Training ===\n')
    rows = load_data()
    X, y = build_arrays(rows)
    print_summary(X, y)
    model, scaler = train(X, y)
    save_model(model, scaler)
    print('Done! Deploy with parking_policy_node.')
