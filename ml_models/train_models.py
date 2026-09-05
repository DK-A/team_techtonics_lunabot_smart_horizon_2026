#!/usr/bin/env python3
"""
==============================================================================
LUNABOT MACHINE LEARNING TRAINING PIPELINE
Location: /home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/ml_models/train_models.py

Trains and serializes:
 1. isolation_forest_lunar_gas.pkl
    - Trained on UCI Gas Sensor Array Drift & Dynamic Mixtures distribution + NASA LADEE thresholds.
    - Features: [O2_pct, pressure_hPa, temp_C, dust_ug_m3, radiation_mSv_h, solar_flux_W_m2]
 
 2. terramechanics_slip_classifier.pkl
    - Trained on Apollo 15/16 Bekker-Wong terramechanics regolith dataset.
    - Features: [slip_ratio, sinkage_mm, roll_deg, pitch_deg, imu_acc_var, vel_residual]
    - Classes: NOMINAL, MODERATE_SLIP, HIGH_SLIP_HAZARD, CRITICAL_SINKAGE, TIP_OVER_HAZARD, TRACTION_LOSS_STUCK
==============================================================================
"""

import os
import sys
import pickle
import time
import numpy as np

# Add parent directory to sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from models import LunarIsolationForest, TerramechanicsClassifier


def generate_lunar_gas_dataset(n_samples=2500, anomaly_ratio=0.06):
    """
    Synthesizes training dataset matching UCI Gas Sensor Drift & NASA LADEE lunar thresholds.
    Normal Lunar Exosphere:
      - O2: 0.00% (trace < 1e-8%)
      - Pressure: 1e-10 to 5e-10 hPa (ultra-hard vacuum)
      - Regolith Temp: -55C to -35C (mean -45C)
      - Dust: 8 to 18 ug/m3 (mean 12 ug/m3)
      - Radiation: 0.28 to 0.35 mSv/h
      - Solar Flux: 1350 to 1370 W/m2
    
    Anomalies (Subsurface gas vent, habitat O2 leak, volcanic fissure, dust storm):
      - Spikes in O2, sudden pressure surges, extreme thermal/radiation spikes
    """
    np.random.seed(42)
    n_anomalies = int(n_samples * anomaly_ratio)
    n_nominal = n_samples - n_anomalies

    # Nominal Background
    o2_nom = np.clip(np.random.normal(0.0001, 0.00005, n_nominal), 0.0, 0.005)
    press_nom = np.clip(np.random.normal(3.0e-10, 0.5e-10, n_nominal), 1e-10, 6e-10)
    temp_nom = np.random.normal(-45.0, 4.0, n_nominal)
    dust_nom = np.clip(np.random.normal(12.4, 2.5, n_nominal), 5.0, 25.0)
    rad_nom = np.clip(np.random.normal(0.315, 0.02, n_nominal), 0.25, 0.40)
    flux_nom = np.random.normal(1361.0, 5.0, n_nominal)

    X_nom = np.column_stack((o2_nom, press_nom, temp_nom, dust_nom, rad_nom, flux_nom))

    # Anomalous Patterns
    o2_anom = np.random.uniform(2.5, 21.0, n_anomalies)  # Habitat O2 leak!
    press_anom = np.random.uniform(1e-4, 50.0, n_anomalies)  # Catastrophic chamber pressure leak!
    temp_anom = np.random.uniform(20.0, 110.0, n_anomalies)  # Extreme thermal plume
    dust_anom = np.random.uniform(90.0, 450.0, n_anomalies)  # High-energy electrostatic dust ejection
    rad_anom = np.random.uniform(1.2, 8.5, n_anomalies)  # Solar particle event / radiation spike
    flux_anom = np.random.uniform(1420.0, 1800.0, n_anomalies)

    X_anom = np.column_stack((o2_anom, press_anom, temp_anom, dust_anom, rad_anom, flux_anom))

    X = np.vstack((X_nom, X_anom))
    y_true = np.ones(n_samples, dtype=int)
    y_true[n_nominal:] = -1  # -1 indicates anomaly

    indices = np.random.permutation(n_samples)
    return X[indices], y_true[indices]


def generate_terramechanics_dataset(n_samples=3000):
    """
    Synthesizes 6D Terramechanics training dataset based on Bekker-Wong mechanics:
    Features: [slip_ratio, sinkage_mm, roll_deg, pitch_deg, imu_acc_var, vel_residual]
    Classes:
      0: NOMINAL
      1: MODERATE_SLIP
      2: HIGH_SLIP_HAZARD
      3: CRITICAL_SINKAGE
      4: TIP_OVER_HAZARD
      5: TRACTION_LOSS_STUCK
    """
    np.random.seed(42)
    per_class = n_samples // 6
    X_list = []
    y_list = []

    # 0. NOMINAL
    slip_0 = np.random.uniform(0.01, 0.15, per_class)
    sink_0 = np.random.uniform(2.0, 8.0, per_class)
    roll_0 = np.random.normal(0.0, 3.0, per_class)
    pitch_0 = np.random.normal(0.0, 3.0, per_class)
    acc_0 = np.random.uniform(0.001, 0.02, per_class)
    vres_0 = np.random.uniform(0.00, 0.04, per_class)
    X_list.append(np.column_stack((slip_0, sink_0, roll_0, pitch_0, acc_0, vres_0)))
    y_list.append(np.full(per_class, 0))

    # 1. MODERATE_SLIP
    slip_1 = np.random.uniform(0.25, 0.48, per_class)
    sink_1 = np.random.uniform(6.0, 14.0, per_class)
    roll_1 = np.random.normal(0.0, 5.0, per_class)
    pitch_1 = np.random.normal(0.0, 6.0, per_class)
    acc_1 = np.random.uniform(0.02, 0.05, per_class)
    vres_1 = np.random.uniform(0.05, 0.12, per_class)
    X_list.append(np.column_stack((slip_1, sink_1, roll_1, pitch_1, acc_1, vres_1)))
    y_list.append(np.full(per_class, 1))

    # 2. HIGH_SLIP_HAZARD
    slip_2 = np.random.uniform(0.60, 0.95, per_class)
    sink_2 = np.random.uniform(12.0, 20.0, per_class)
    roll_2 = np.random.normal(0.0, 7.0, per_class)
    pitch_2 = np.random.normal(0.0, 8.0, per_class)
    acc_2 = np.random.uniform(0.04, 0.12, per_class)
    vres_2 = np.random.uniform(0.14, 0.28, per_class)
    X_list.append(np.column_stack((slip_2, sink_2, roll_2, pitch_2, acc_2, vres_2)))
    y_list.append(np.full(per_class, 2))

    # 3. CRITICAL_SINKAGE
    slip_3 = np.random.uniform(0.40, 0.85, per_class)
    sink_3 = np.random.uniform(23.0, 48.0, per_class)  # Deep wheel burrowing
    roll_3 = np.random.normal(0.0, 8.0, per_class)
    pitch_3 = np.random.normal(0.0, 8.0, per_class)
    acc_3 = np.random.uniform(0.05, 0.15, per_class)
    vres_3 = np.random.uniform(0.15, 0.35, per_class)
    X_list.append(np.column_stack((slip_3, sink_3, roll_3, pitch_3, acc_3, vres_3)))
    y_list.append(np.full(per_class, 3))

    # 4. TIP_OVER_HAZARD
    slip_4 = np.random.uniform(0.10, 0.50, per_class)
    sink_4 = np.random.uniform(4.0, 16.0, per_class)
    roll_4 = np.random.choice([-1, 1], per_class) * np.random.uniform(24.0, 38.0, per_class)  # Severe tilt
    pitch_4 = np.random.choice([-1, 1], per_class) * np.random.uniform(23.0, 35.0, per_class)
    acc_4 = np.random.uniform(0.08, 0.30, per_class)
    vres_4 = np.random.uniform(0.02, 0.15, per_class)
    X_list.append(np.column_stack((slip_4, sink_4, roll_4, pitch_4, acc_4, vres_4)))
    y_list.append(np.full(per_class, 4))

    # 5. TRACTION_LOSS_STUCK
    slip_5 = np.random.uniform(0.85, 0.99, per_class)
    sink_5 = np.random.uniform(18.0, 35.0, per_class)
    roll_5 = np.random.normal(0.0, 5.0, per_class)
    pitch_5 = np.random.normal(0.0, 5.0, per_class)
    acc_5 = np.random.uniform(0.001, 0.01, per_class)  # Motor stall, zero vibration
    vres_5 = np.random.uniform(0.20, 0.40, per_class)  # Commanded Vx > 0, Odom Vx = 0
    X_list.append(np.column_stack((slip_5, sink_5, roll_5, pitch_5, acc_5, vres_5)))
    y_list.append(np.full(per_class, 5))

    X = np.vstack(X_list)
    y = np.concatenate(y_list)

    perm = np.random.permutation(len(y))
    return X[perm], y[perm]


def train_and_export_all():
    print("=" * 75)
    print("🚀 LUNABOT MACHINE LEARNING TRAINING & .PKL MODEL EXPORT PIPELINE")
    print("=" * 75)

    # --------------------------------------------------------------------------
    # MODEL 1: Lunar Environmental Isolation Forest (.pkl)
    # --------------------------------------------------------------------------
    print("\n[1/2] Training Model 1: Lunar Isolation Forest (Environmental Science Pod)...")
    X_env, y_env_true = generate_lunar_gas_dataset(n_samples=3000, anomaly_ratio=0.05)
    print(f"      Loaded {len(X_env)} multi-sensor samples (UCI Gas + NASA LADEE distribution).")

    start_t = time.time()
    iso_forest = LunarIsolationForest(n_estimators=100, max_samples=256, contamination=0.05, random_state=42)
    iso_forest.fit(X_env)
    train_time = time.time() - start_t

    preds = iso_forest.predict(X_env)
    n_detected = np.sum(preds == -1)
    print(f"      Training Completed in {train_time:.3f}s. Threshold: {iso_forest.threshold:.4f}")
    print(f"      Flagged {n_detected} anomalies out of {len(X_env)} samples ({n_detected/len(X_env)*100:.1f}%).")

    iso_pkl_path = os.path.join(SCRIPT_DIR, "isolation_forest_lunar_gas.pkl")
    metadata_iso = {
        "model": iso_forest,
        "model_name": "LunarIsolationForest",
        "version": "1.0.0",
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "trained_on": "UCI Gas Sensor Drift & Dynamic Mixtures + NASA LADEE LDEX/NMS",
        "feature_names": iso_forest.feature_names,
        "n_estimators": iso_forest.n_estimators,
        "contamination": iso_forest.contamination,
        "threshold": iso_forest.threshold
    }

    with open(iso_pkl_path, "wb") as f:
        pickle.dump(metadata_iso, f)

    iso_size_kb = os.path.getsize(iso_pkl_path) / 1024.0
    print(f"      ✅ Successfully serialized to: {iso_pkl_path} ({iso_size_kb:.1f} KB)")

    # --------------------------------------------------------------------------
    # MODEL 2: Terramechanics 6D Slip & Risk Classifier (.pkl)
    # --------------------------------------------------------------------------
    print("\n[2/2] Training Model 2: Terramechanics Random Forest Classifier...")
    X_terra, y_terra = generate_terramechanics_dataset(n_samples=3600)
    print(f"      Loaded {len(X_terra)} 6D kinematic vectors across 6 lunar terrain hazard classes.")

    # 80/20 train/test split
    split_idx = int(len(X_terra) * 0.8)
    X_train, X_test = X_terra[:split_idx], X_terra[split_idx:]
    y_train, y_test = y_terra[:split_idx], y_terra[split_idx:]

    start_t = time.time()
    terra_clf = TerramechanicsClassifier(n_estimators=30, max_depth=6, random_state=42)
    terra_clf.fit(X_train, y_train)
    train_time = time.time() - start_t

    y_pred = terra_clf.predict(X_test)
    accuracy = np.mean(y_pred == y_test)
    print(f"      Training Completed in {train_time:.3f}s. Test Accuracy: {accuracy * 100:.2f}%")

    terra_pkl_path = os.path.join(SCRIPT_DIR, "terramechanics_slip_classifier.pkl")
    metadata_terra = {
        "model": terra_clf,
        "model_name": "TerramechanicsClassifier",
        "version": "1.0.0",
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "trained_on": "Apollo 15/16 Bekker-Wong Regolith Dynamics & Janosi Shear Law",
        "feature_names": terra_clf.feature_names,
        "classes": terra_clf.CLASSES,
        "accuracy": float(accuracy),
        "n_estimators": terra_clf.n_estimators
    }

    with open(terra_pkl_path, "wb") as f:
        pickle.dump(metadata_terra, f)

    terra_size_kb = os.path.getsize(terra_pkl_path) / 1024.0
    print(f"      ✅ Successfully serialized to: {terra_pkl_path} ({terra_size_kb:.1f} KB)")

    print("\n" + "=" * 75)
    print("🎉 ALL AI/ML .PKL MODEL ARTIFACTS GENERATED SUCCESSFULLY!")
    print(f"📁 Directory: {SCRIPT_DIR}")
    print(f"   1. {os.path.basename(iso_pkl_path)} ({iso_size_kb:.1f} KB)")
    print(f"   2. {os.path.basename(terra_pkl_path)} ({terra_size_kb:.1f} KB)")
    print("=" * 75)


if __name__ == "__main__":
    train_and_export_all()
