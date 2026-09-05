"""
==============================================================================
LUNABOT MACHINE LEARNING MODEL DEFINITIONS
Location: /home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/ml_models/models.py

Contains:
 1. LunarIsolationForest:
    - Pure NumPy implementation of Liu et al. (2008) Isolation Forest algorithm.
    - Zero external dependency footprint for embedded real-time robotics.
    - Methods: fit(), predict(), score_samples(), decision_function().
    - Fully serializable via pickle into .pkl.

 2. TerramechanicsRandomForest:
    - Decision Tree Ensemble for 6D Terramechanics Risk Classification.
    - Classes: NOMINAL, MODERATE_SLIP, HIGH_SLIP_HAZARD, CRITICAL_SINKAGE, TIP_OVER_HAZARD, TRACTION_LOSS_STUCK.
    - Methods: fit(), predict(), predict_proba().
==============================================================================
"""

import math
import numpy as np


class IsolationTreeNode:
    """A single node in an Isolation Tree (iTree)."""
    def __init__(self, left=None, right=None, split_feature=None, split_val=None, size=None):
        self.left = left
        self.right = right
        self.split_feature = split_feature
        self.split_val = split_val
        self.size = size
        self.is_leaf = (left is None and right is None)


class IsolationTree:
    """An individual Isolation Tree."""
    def __init__(self, max_depth):
        self.max_depth = max_depth
        self.root = None

    def fit(self, X, current_depth=0):
        n_samples, n_features = X.shape
        if current_depth >= self.max_depth or n_samples <= 1:
            return IsolationTreeNode(size=n_samples)

        feat_idx = np.random.randint(0, n_features)
        feat_min = X[:, feat_idx].min()
        feat_max = X[:, feat_idx].max()

        if feat_min == feat_max:
            return IsolationTreeNode(size=n_samples)

        split_val = np.random.uniform(feat_min, feat_max)
        left_mask = X[:, feat_idx] < split_val
        right_mask = ~left_mask

        if np.sum(left_mask) == 0 or np.sum(right_mask) == 0:
            return IsolationTreeNode(size=n_samples)

        left_node = self.fit(X[left_mask], current_depth + 1)
        right_node = self.fit(X[right_mask], current_depth + 1)

        return IsolationTreeNode(
            left=left_node,
            right=right_node,
            split_feature=feat_idx,
            split_val=split_val,
            size=n_samples
        )

    def path_length(self, x, node, current_depth=0):
        if node.is_leaf:
            return current_depth + self._c(node.size)
        
        if x[node.split_feature] < node.split_val:
            return self.path_length(x, node.left, current_depth + 1)
        else:
            return self.path_length(x, node.right, current_depth + 1)

    @staticmethod
    def _c(n):
        if n <= 1:
            return 0.0
        if n == 2:
            return 1.0
        euler_constant = 0.5772156649
        return 2.0 * (math.log(n - 1) + euler_constant) - (2.0 * (n - 1) / n)


class LunarIsolationForest:
    """
    Isolation Forest for Lunar Environmental & Chemical Sensor Anomaly Detection.
    Trained on simulated multi-sensor gas mixtures (UCI Gas Drift & NASA LADEE benchmarks).
    """
    def __init__(self, n_estimators=100, max_samples=256, contamination=0.05, random_state=42):
        self.n_estimators = n_estimators
        self.max_samples = max_samples
        self.contamination = contamination
        self.random_state = random_state
        self.trees = []
        self.threshold = 0.50
        self.feature_names = [
            "O2_Concentration_pct",
            "Chamber_Pressure_hPa",
            "Regolith_Temp_C",
            "Dust_Concentration_ug_m3",
            "Radiation_mSv_h",
            "Solar_Flux_W_m2"
        ]

    def fit(self, X):
        np.random.seed(self.random_state)
        n_samples = X.shape[0]
        subsample_size = min(self.max_samples, n_samples)
        max_depth = int(math.ceil(math.log2(max(subsample_size, 2))))

        self.trees = []
        for _ in range(self.n_estimators):
            idx = np.random.choice(n_samples, subsample_size, replace=False)
            tree = IsolationTree(max_depth=max_depth)
            tree.root = tree.fit(X[idx])
            self.trees.append(tree)

        scores = self.score_samples(X)
        self.threshold = float(np.percentile(scores, 100 * (1.0 - self.contamination)))
        return self

    def score_samples(self, X):
        X = np.atleast_2d(X)
        n_samples = X.shape[0]
        scores = np.zeros(n_samples)

        subsample_size = self.max_samples
        c_val = IsolationTree._c(subsample_size)

        for i in range(n_samples):
            x = X[i]
            avg_path = np.mean([tree.path_length(x, tree.root) for tree in self.trees])
            scores[i] = 2.0 ** (-avg_path / max(c_val, 1e-6))

        return scores

    def decision_function(self, X):
        return self.threshold - self.score_samples(X)

    def predict(self, X):
        scores = self.score_samples(X)
        preds = np.ones(len(scores), dtype=int)
        preds[scores >= self.threshold] = -1
        return preds


class TerramechanicsDecisionTree:
    """Fast Decision Tree for multi-class classification."""
    def __init__(self, max_depth=6, min_samples_split=5):
        self.max_depth = max_depth
        self.min_samples_split = min_samples_split
        self.tree = None

    def fit(self, X, y, depth=0):
        n_samples, n_features = X.shape
        n_classes = len(np.unique(y))

        if depth >= self.max_depth or n_classes <= 1 or n_samples < self.min_samples_split:
            counts = np.bincount(y, minlength=6)
            return {'is_leaf': True, 'class': int(np.argmax(counts)), 'probs': (counts / max(1, n_samples)).tolist()}

        best_gini = 1.0
        best_split = None

        for feat in range(n_features):
            thresholds = np.percentile(X[:, feat], [20, 40, 60, 80])
            for th in thresholds:
                left_mask = X[:, feat] <= th
                right_mask = ~left_mask
                if np.sum(left_mask) == 0 or np.sum(right_mask) == 0:
                    continue

                gini = (np.sum(left_mask) * self._gini(y[left_mask]) + 
                        np.sum(right_mask) * self._gini(y[right_mask])) / n_samples
                if gini < best_gini:
                    best_gini = gini
                    best_split = (feat, th, left_mask, right_mask)

        if best_split is None or best_gini >= self._gini(y):
            counts = np.bincount(y, minlength=6)
            return {'is_leaf': True, 'class': int(np.argmax(counts)), 'probs': (counts / max(1, n_samples)).tolist()}

        feat, th, left_mask, right_mask = best_split
        left_sub = self.fit(X[left_mask], y[left_mask], depth + 1)
        right_sub = self.fit(X[right_mask], y[right_mask], depth + 1)

        return {
            'is_leaf': False,
            'feature': feat,
            'threshold': th,
            'left': left_sub,
            'right': right_sub
        }

    @staticmethod
    def _gini(y):
        if len(y) == 0:
            return 0.0
        counts = np.bincount(y)
        probs = counts / len(y)
        return float(1.0 - np.sum(probs ** 2))

    def _predict_row(self, x, node):
        if node['is_leaf']:
            return node['class'], node['probs']
        if x[node['feature']] <= node['threshold']:
            return self._predict_row(x, node['left'])
        else:
            return self._predict_row(x, node['right'])


class TerramechanicsClassifier:
    """
    Ensemble Classifier for 6D Terramechanics Risk Assessment:
    Features: [slip_ratio, sinkage_mm, roll_deg, pitch_deg, acc_var, vel_residual]
    """
    CLASSES = [
        "NOMINAL",
        "MODERATE_SLIP",
        "HIGH_SLIP_HAZARD",
        "CRITICAL_SINKAGE",
        "TIP_OVER_HAZARD",
        "TRACTION_LOSS_STUCK"
    ]

    def __init__(self, n_estimators=25, max_depth=6, random_state=42):
        self.n_estimators = n_estimators
        self.max_depth = max_depth
        self.random_state = random_state
        self.trees = []
        self.feature_names = [
            "slip_ratio",
            "sinkage_mm",
            "roll_deg",
            "pitch_deg",
            "imu_acc_var",
            "vel_residual"
        ]

    def fit(self, X, y):
        np.random.seed(self.random_state)
        n_samples = X.shape[0]
        self.trees = []
        for _ in range(self.n_estimators):
            idx = np.random.choice(n_samples, n_samples, replace=True)
            tree = TerramechanicsDecisionTree(max_depth=self.max_depth)
            tree.tree = tree.fit(X[idx], y[idx])
            self.trees.append(tree)
        return self

    def predict_proba(self, X):
        X = np.atleast_2d(X)
        all_probs = []
        for x in X:
            tree_probs = [tree._predict_row(x, tree.tree)[1] for tree in self.trees]
            avg_prob = np.mean(tree_probs, axis=0)
            all_probs.append(avg_prob)
        return np.array(all_probs)

    def predict(self, X):
        probs = self.predict_proba(X)
        return np.argmax(probs, axis=1)

    def predict_class_name(self, x):
        cls_idx = self.predict(x)[0]
        return self.CLASSES[cls_idx]
