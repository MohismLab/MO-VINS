#!/usr/bin/env python3
"""Evaluate metric body trajectories against timestamped camera poses (SE(3), no scale fit)."""
import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np


def align(source, target):
    """Return proper rigid rotation and translation mapping source onto target."""
    a, b = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source-a).T @ (target-b))
    sign = np.eye(3)
    sign[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ sign @ u.T
    return rotation, b-rotation @ a


def quaternion_rotation(q):
    q = np.asarray(q, dtype=float)
    if not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > .01:
        raise ValueError('invalid reference quaternion')
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def statistics(values):
    if not len(values):
        return None
    x = np.asarray(values)
    return dict(count=len(x), rmse=float(np.sqrt(np.mean(x*x))),
                median=float(np.median(x)), p95=float(np.percentile(x, 95)), max=float(x.max()))


def evaluate(rows, reference, rbc, tbc, max_gap=.11, interval=1.):
    gt = np.asarray(reference, dtype=float)
    if gt.ndim != 2 or gt.shape[1] != 8 or len(gt) < 3 or not np.isfinite(gt).all():
        raise ValueError('expected timestamp_s,x,y,z,qw,qx,qy,qz reference')
    if np.any(np.diff(gt[:, 0]) <= 0):
        raise ValueError('reference timestamps must increase')
    gr = [quaternion_rotation(row[4:]) for row in gt]
    segments, current = [], []
    previous = None
    valid = 0
    for row in rows:
        stamp = float(row['timestamp_s'])
        if previous is not None and stamp <= previous[0]:
            raise ValueError('trajectory timestamps must increase')
        epoch = int(row['reset_counter'])
        ok = int(row['valid']) != 0
        valid += ok
        if not ok or (previous is not None and (epoch != previous[1] or stamp-previous[0] > .2)):
            if current:
                segments.append(current)
                current = []
        previous = stamp, epoch
        if not ok:
            continue
        p = np.array([float(row[k]) for k in ('x', 'y', 'z')])
        rv = np.array([float(row[k]) for k in ('rx', 'ry', 'rz')])
        if not np.isfinite(p).all() or not np.isfinite(rv).all():
            raise ValueError('valid pose contains nonfinite values')
        rotation = cv2.Rodrigues(rv)[0]
        p = p + rotation @ tbc
        rotation = rotation @ rbc
        j = int(np.searchsorted(gt[:, 0], stamp, side='right'))
        if j == 0 or j == len(gt) or gt[j, 0]-gt[j-1, 0] > max_gap:
            continue
        alpha = (stamp-gt[j-1, 0])/(gt[j, 0]-gt[j-1, 0])
        # Geodesic SO(3) interpolation is equivalent to quaternion SLERP.
        delta = cv2.Rodrigues(gr[j-1].T @ gr[j])[0]
        rg = gr[j-1] @ cv2.Rodrigues(alpha*delta)[0]
        pg = (1-alpha)*gt[j-1, 1:4]+alpha*gt[j, 1:4]
        current.append((stamp, p, rotation, pg, rg))
    if current:
        segments.append(current)
    results, errors, translation_rpe, rotation_rpe = [], [], [], []
    for segment in segments:
        if len(segment) < 3:
            continue
        p = np.array([item[1] for item in segment])
        g = np.array([item[3] for item in segment])
        rotation, translation = align(p, g)
        error = np.linalg.norm(p @ rotation.T+translation-g, axis=1)
        errors.extend(error)
        results.append(dict(start_s=segment[0][0], end_s=segment[-1][0],
                            duration_s=segment[-1][0]-segment[0][0], ate_m=statistics(error)))
        stamps = np.array([item[0] for item in segment])
        for i, a in enumerate(segment):
            j = int(np.searchsorted(stamps, a[0]+interval))
            if j == len(segment) or abs(stamps[j]-a[0]-interval) > .06:
                continue
            b = segment[j]
            dp = a[2].T @ (b[1]-a[1])
            dg = a[4].T @ (b[3]-a[3])
            translation_rpe.append(float(np.linalg.norm(dp-dg)))
            dr = (a[4].T @ b[4]).T @ (a[2].T @ b[2])
            rotation_rpe.append(float(np.linalg.norm(cv2.Rodrigues(dr)[0])*180/np.pi))
    return dict(frames=len(rows), valid_frames=valid, valid_fraction=valid/max(1, len(rows)),
                matched_frames=sum(len(s) for s in segments),
                reset_counter_changes=sum(int(a['reset_counter']) != int(b['reset_counter']) for a,b in zip(rows, rows[1:])),
                alignment='SE(3), no scale estimation; independent alignment per contiguous valid epoch',
                full_sequence_ate_m=statistics(errors) if len(segments) == 1 else None,
                segmented_ate_m=statistics(errors), segments=results,
                rpe_interval_s=interval, rpe_translation_m=statistics(translation_rpe),
                rpe_rotation_deg=statistics(rotation_rpe),
                statuses=dict(Counter(row['status'] for row in rows)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory')
    parser.add_argument('reference', help='EuRoC left-camera CSV: ns,x,y,z,qw,qx,qy,qz')
    parser.add_argument('calibration')
    parser.add_argument('output')
    parser.add_argument('--reference-frame', choices=('camera', 'imu'), default='camera')
    args = parser.parse_args()
    with open(args.trajectory) as stream:
        rows = list(csv.DictReader(stream))
    gt = np.loadtxt(args.reference, delimiter=',', comments='#')[:, :8]
    gt[:, 0] *= 1e-9
    config = cv2.FileStorage(args.calibration, cv2.FILE_STORAGE_READ)
    if not config.isOpened():
        raise ValueError('cannot open calibration')
    rbc, tbc = config.getNode('Rbc').mat(), config.getNode('tbc').mat().reshape(3)
    rbi = config.getNode('Rbi').mat()
    config.release()
    if args.reference_frame == 'imu':
        # The native IMU ground truth retains its orientation. Some derived
        # camera-position reference files do not transform quaternion columns.
        ric, tic = rbi.T @ rbc, rbi.T @ tbc
        for row in gt:
            ri = quaternion_rotation(row[4:])
            row[1:4] += ri @ tic
            rc = ri @ ric
            # Stable conversion through axis-angle, returning w,x,y,z.
            rv = cv2.Rodrigues(rc)[0].reshape(3)
            angle = np.linalg.norm(rv)
            row[4:] = np.r_[np.cos(angle/2), rv*(np.sin(angle/2)/angle if angle > 1e-12 else .5)]
    result = evaluate(rows, gt, rbc, tbc)
    Path(args.output).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key:value for key,value in result.items() if key not in ('segments','statuses')}, indent=2))


if __name__ == '__main__':
    main()
