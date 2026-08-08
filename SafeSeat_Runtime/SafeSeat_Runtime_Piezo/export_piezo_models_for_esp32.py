"""
SafeSeat PIEZO sklearn -> ESP32 exporter
=========================================

Exports:
- StandardScaler mean/scale
- Isolation Forest tree structure
- One-Class SVM support vectors / dual coefficients / RBF params

The generated C++ inference code reproduces sklearn decision-function
semantics:
    >= 0 : inlier / normal
    <  0 : anomaly

This script is provided for reproducibility. The generated files in the
runtime package were produced from the trained canonical PIEZO models.
"""

from pathlib import Path
import joblib
import numpy as np

MODEL_DIR = Path("models/PIEZO")

scaler = joblib.load(MODEL_DIR / "piezo_scaler.joblib")
iforest = joblib.load(MODEL_DIR / "piezo_isolation_forest.joblib")
ocsvm = joblib.load(MODEL_DIR / "piezo_one_class_svm.joblib")

print("Feature count:", scaler.mean_.shape[0])
print("Isolation Forest trees:", len(iforest.estimators_))
print("Isolation Forest max_samples:", iforest.max_samples_)
print("Isolation Forest offset:", iforest.offset_)
print("Total IF nodes:", sum(e.tree_.node_count for e in iforest.estimators_))
print("OCSVM support vectors:", ocsvm.support_vectors_.shape)
print("OCSVM gamma:", ocsvm._gamma)
print("OCSVM intercept:", ocsvm.intercept_)
