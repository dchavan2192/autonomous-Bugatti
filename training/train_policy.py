#!/usr/bin/env python3
"""
Behavior cloning training script.

Usage:
    python3 train_policy.py

Reads all session_*.csv files from ~/ros2_ws/training_data/,
trains a small MLP, and saves the model to ~/ros2_ws/trained_policy.pkl

No ROS2 needed — run from any terminal.
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

TRAINING_DIR = os.path.expanduser('~/ros2_ws/training_data')
MODEL_PATH   = os.path.expanduser('~/ros2_ws/trained_policy.pkl')

FEATURE_COLS = ['sonar_cm', 'sonar_trend', 'obj_left', 'obj_center', 'obj_right', 'person_present']
LABEL_COLS   = ['speed', 'steering']

# Normalization constants (match policy_node.py)
SONAR_MAX    = 200.0
SPEED_MAX    = 255.0
STEER_MAX    = 90.0


def load_data():
    files = sorted(glob.glob(os.path.join(TRAINING_DIR, 'session_*.csv')))
    if not files:
        raise FileNotFoundError(f'No session CSV files found in {TRAINING_DIR}')

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
            sonar  = float(row['sonar_cm'])
            trend  = float(row['sonar_trend'])
            o_l    = float(row['obj_left'])
            o_c    = float(row['obj_center'])
            o_r    = float(row['obj_right'])
            person = float(row['person_present'])
            speed  = float(row['speed'])
            steer  = float(row['steering'])
        except (ValueError, KeyError):
            skipped += 1
            continue

        # Normalize features
        sonar_norm = min(sonar, SONAR_MAX) / SONAR_MAX  # 0=close, 1=far
        trend_norm = (trend + 1) / 2.0                  # -1→0, 0→0.5, 1→1

        features = [sonar_norm, trend_norm, o_l / 5.0, o_c / 5.0, o_r / 5.0, person]
        labels   = [speed / SPEED_MAX, steer / STEER_MAX]

        X.append(features)
        y.append(labels)

    if skipped:
        print(f'Skipped {skipped} malformed rows.')

    return np.array(X, dtype=np.float32), np.array(y, dtype=np.float32)


def print_dataset_summary(X, y):
    speeds   = y[:, 0] * SPEED_MAX
    steerings = y[:, 1] * STEER_MAX

    print(f'\nDataset summary ({len(X)} samples):')
    print(f'  Speed    — mean: {speeds.mean():.1f}  min: {speeds.min():.1f}  max: {speeds.max():.1f}')
    print(f'  Steering — mean: {steerings.mean():.1f}  min: {steerings.min():.1f}  max: {steerings.max():.1f}')
    fwd  = (speeds > 10).sum()
    stop = (np.abs(speeds) < 10).sum()
    back = (speeds < -10).sum()
    print(f'  Forward: {fwd} ({100*fwd/len(X):.0f}%)  Stop: {stop} ({100*stop/len(X):.0f}%)  Backward: {back} ({100*back/len(X):.0f}%)')
    left  = (steerings < -5).sum()
    straight = (np.abs(steerings) <= 5).sum()
    right = (steerings > 5).sum()
    print(f'  Left: {left} ({100*left/len(X):.0f}%)  Straight: {straight} ({100*straight/len(X):.0f}%)  Right: {right} ({100*right/len(X):.0f}%)')


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
        verbose=False,
        early_stopping=True,
        validation_fraction=0.1,
        n_iter_no_change=20,
    )
    model.fit(X_train_s, y_train)

    preds = model.predict(X_val_s)
    speed_mae   = mean_absolute_error(y_val[:, 0], preds[:, 0]) * SPEED_MAX
    steer_mae   = mean_absolute_error(y_val[:, 1], preds[:, 1]) * STEER_MAX

    print(f'Validation MAE — speed: {speed_mae:.1f} (out of 255)  steering: {steer_mae:.1f}° (out of 90°)')
    print(f'Training stopped after {model.n_iter_} iterations.')

    return model, scaler


def save_model(model, scaler):
    payload = {'model': model, 'scaler': scaler}
    with open(MODEL_PATH, 'wb') as f:
        pickle.dump(payload, f)
    print(f'\nModel saved to {MODEL_PATH}')


if __name__ == '__main__':
    print('=== Behavior Cloning Training ===\n')
    rows = load_data()
    X, y = build_arrays(rows)
    print_dataset_summary(X, y)
    model, scaler = train(X, y)
    save_model(model, scaler)
    print('\nDone! Run policy_node to deploy.')
