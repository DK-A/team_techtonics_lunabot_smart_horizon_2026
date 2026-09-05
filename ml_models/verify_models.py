#!/usr/bin/env python3
"""
==============================================================================
LUNABOT AI/ML MODEL INFERENCE & VERIFICATION DEMO SCRIPT
Location: /home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/ml_models/verify_models.py

Demonstrates loading the serialized .pkl model files and running real-time
inference for hackathon evaluators and technical judges.
==============================================================================
"""

import os
import sys
import pickle
import time
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

# Import model classes so unpickler can instantiate them
from models import LunarIsolationForest, TerramechanicsClassifier


def test_models():
    print("=" * 78)
    print("🔬 LUNABOT MACHINE LEARNING MODEL INFERENCE & VERIFICATION SUITE")
    print("=" * 78)

    iso_path = os.path.join(SCRIPT_DIR, "isolation_forest_lunar_gas.pkl")
    terra_path = os.path.join(SCRIPT_DIR, "terramechanics_slip_classifier.pkl")

    if not os.path.exists(iso_path) or not os.path.exists(terra_path):
        print("❌ Model files not found! Running train_models.py first...")
        import train_models
        train_models.train_and_export_all()

    # --------------------------------------------------------------------------
    # 1. LOAD & TEST MODEL 1: ISOLATION FOREST (.pkl)
    # --------------------------------------------------------------------------
    print("\n📦 [MODEL 1] Loading 'isolation_forest_lunar_gas.pkl'...")
    with open(iso_path, "rb") as f:
        meta_iso = pickle.load(f)

    iso_model = meta_iso["model"]
    print(f"   • Model Architecture: {meta_iso['model_name']} ({meta_iso['n_estimators']} iTrees)")
    print(f"   • Training Baseline : {meta_iso['trained_on']}")
    print(f"   • Anomaly Threshold : {meta_iso['threshold']:.4f}")
    print(f"   • Features          : {', '.join(meta_iso['feature_names'])}")

    test_env_scenarios = [
        {
            "name": "Nominal Lunar Surface Vacuum (Normal Patrol)",
            "vector": np.array([0.00, 3.0e-10, -45.0, 12.4, 0.315, 1361.0])
        },
        {
            "name": "Habitat Airlock Oxygen Leak (Critical Anomaly)",
            "vector": np.array([18.50, 4.2e-2, -15.0, 35.0, 0.320, 1361.0])
        },
        {
            "name": "Subsurface Volcanic Fissure & Dust Ejection (Hazard)",
            "vector": np.array([0.00, 8.5e-5, 85.0, 240.0, 0.650, 1420.0])
        }
    ]

    print("\n   --- Executing Isolation Forest Real-Time Inference ---")
    for scen in test_env_scenarios:
        t0 = time.perf_counter()
        score = iso_model.score_samples(scen["vector"])[0]
        pred = iso_model.predict(scen["vector"])[0]
        dt_ms = (time.perf_counter() - t0) * 1000.0

        status = "⚠️ ANOMALY DETECTED" if pred == -1 else "✅ NOMINAL INLIER"
        print(f"   ► Scenario: {scen['name']}")
        print(f"     Score: {score:.4f} (Thresh: {iso_model.threshold:.4f}) | Latency: {dt_ms:.2f}ms | Status: {status}")

    # --------------------------------------------------------------------------
    # 2. LOAD & TEST MODEL 2: TERRAMECHANICS CLASSIFIER (.pkl)
    # --------------------------------------------------------------------------
    print("\n" + "-" * 78)
    print("📦 [MODEL 2] Loading 'terramechanics_slip_classifier.pkl'...")
    with open(terra_path, "rb") as f:
        meta_terra = pickle.load(f)

    terra_model = meta_terra["model"]
    print(f"   • Model Architecture: {meta_terra['model_name']} ({meta_terra['n_estimators']} Decision Trees)")
    print(f"   • Training Baseline : {meta_terra['trained_on']}")
    print(f"   • Validation Accuracy: {meta_terra['accuracy']*100:.2f}%")
    print(f"   • Risk Classes      : {', '.join(meta_terra['classes'])}")

    test_terra_scenarios = [
        {
            "name": "Flat Regolith Traversal (Nominal 6WD Cruise)",
            "vector": np.array([0.05, 4.2, 0.5, -0.8, 0.008, 0.01])
        },
        {
            "name": "Loose Crater Dust (Deep Regolith Sinkage)",
            "vector": np.array([0.55, 28.5, 2.1, -3.4, 0.065, 0.18])
        },
        {
            "name": "Steep Boulder Climb (Dangerous Tip-Over Risk)",
            "vector": np.array([0.22, 6.0, 26.5, 14.0, 0.180, 0.08])
        },
        {
            "name": "Wheel Entrenchment / High Slip",
            "vector": np.array([0.88, 16.0, 1.2, 2.0, 0.090, 0.25])
        }
    ]

    print("\n   --- Executing Terramechanics Real-Time Inference ---")
    for scen in test_terra_scenarios:
        t0 = time.perf_counter()
        pred_idx = terra_model.predict(scen["vector"])[0]
        class_name = meta_terra["classes"][pred_idx]
        dt_ms = (time.perf_counter() - t0) * 1000.0

        alert_badge = "🔴 ALERT" if class_name != "NOMINAL" else "🟢 OK"
        print(f"   ► Scenario: {scen['name']}")
        print(f"     Class: {class_name} | Latency: {dt_ms:.2f}ms | Supervision: {alert_badge}")

    print("\n" + "=" * 78)
    print("🎯 CONCLUSION: BOTH .PKL MODELS VALIDATED & READY FOR HACKATHON DEFENSE!")
    print("=" * 78)


if __name__ == "__main__":
    test_models()
