#!/usr/bin/env python3
"""Check rigid alignment, metric scale errors, discontinuities and lever arms."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'host'))
from evaluate_trajectory import evaluate, align
import cv2
import numpy as np

t = np.arange(0, 5, .05)
p = np.c_[np.sin(t), np.cos(t), t*.2]
r = cv2.Rodrigues(np.array([.2, -.3, .4]))[0]
target = p @ r.T + [2, 3, 4]
rotation, shift = align(p, target)
assert np.max(abs(p @ rotation.T+shift-target)) < 1e-12
rows = [dict(timestamp_s=str(stamp), valid='1', reset_counter='0',
             x=str(v[0]), y=str(v[1]), z=str(v[2]), rx='0', ry='0', rz='0', status='tracking')
        for stamp, v in zip(t, p)]
reference = np.c_[t, p+[.1, -.02, .03], np.ones(len(t)), np.zeros((len(t), 3))]
result = evaluate(rows, reference, np.eye(3), np.array([.1, -.02, .03]))
assert result['full_sequence_ate_m']['rmse'] < 1e-12
assert result['rpe_translation_m']['rmse'] < 1e-12
assert result['rpe_rotation_deg']['rmse'] < 1e-12
scaled = reference.copy()
scaled[:, 1:4] *= 1.1
assert evaluate(rows, scaled, np.eye(3), np.zeros(3))['segmented_ate_m']['rmse'] > .02
for row in rows[50:]:
    row['reset_counter'] = '1'
result = evaluate(rows, reference, np.eye(3), np.array([.1, -.02, .03]))
assert result['full_sequence_ate_m'] is None and len(result['segments']) == 2
assert result['reset_counter_changes'] == 1
print('PASS metric SE(3), no fitted scale, camera lever arm, RPE and reset segmentation')
