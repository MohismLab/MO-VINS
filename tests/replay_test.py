#!/usr/bin/env python3
"""Test real CSV/image ingestion and blackout recovery through the host replay binary."""
import csv
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import cv2
import numpy as np

binary = str(Path(sys.argv[1]).resolve())
with tempfile.TemporaryDirectory() as directory:
    root = Path(directory)
    for sensor in ('cam0', 'cam1', 'imu0'):
        (root / sensor / 'data').mkdir(parents=True)
    config = root / 'calibration.yml'
    fs = cv2.FileStorage(str(config), cv2.FILE_STORAGE_WRITE)
    for name, value in dict(calibrated=1, capture_width=320, capture_height=200).items():
        fs.write(name, value)
    k = np.array([[200.,0,160],[0,200,100],[0,0,1]])
    for name, value in dict(K1=k, K2=k, D1=np.zeros((1,5)), D2=np.zeros((1,5)),
                            R=np.eye(3), Rbc=np.eye(3), Rbi=np.eye(3),
                            T=np.array([[-.06],[0],[0]]), tbc=np.zeros((3,1))).items():
        fs.write(name, value)
    fs.release()
    rng = np.random.default_rng(42)
    image = cv2.GaussianBlur(rng.integers(0,256,(200,320),np.uint8),(3,3),.7)
    right = np.zeros_like(image); right[:,:312] = image[:,8:]
    for sensor, texture in [('cam0',image), ('cam1',right)]:
        with (root / sensor / 'data.csv').open('w') as f:
            f.write('#timestamp [ns],filename\n')
            for i in range(65):
                stamp = 1_000_000_000 + i * 66_666_667
                f.write(f'{stamp},{stamp}.png\n')
                pixels = np.zeros_like(texture) if 20 <= i <= 22 else texture
                assert cv2.imwrite(str(root / sensor / 'data' / f'{stamp}.png'), pixels)
    with (root / 'imu0/data.csv').open('w') as f:
        f.write('#timestamp [ns],wx,wy,wz,ax,ay,az\n')
        for i in range(1900):
            f.write(f'{1_000_000_000 + round(i * 1e9 / 416)},0,0,0,0,0,-9.80665\n')
    output = root / 'trajectory.csv'
    subprocess.run([binary, str(config), str(root), str(output)], check=True)
    with output.open() as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 65
    assert sum(int(row['valid']) for row in rows) > 25
    assert rows[0]['valid'] == '0' and rows[-1]['valid'] == '1'
    assert all(rows[i]['valid'] == '0' for i in range(20,23))
    assert int(rows[-1]['reset_counter']) > 0
    for row in rows:
        assert math.isfinite(float(row['x'])) == bool(int(row['valid']))
    # Complete lack of texture is a failed benchmark, not a successful zero trajectory.
    for sensor in ('cam0', 'cam1'):
        for path in (root / sensor / 'data').glob('*.png'):
            assert cv2.imwrite(str(path), np.zeros_like(image))
    result = subprocess.run([binary, str(config), str(root), str(output)])
    assert result.returncode == 3
print('PASS EuRoC CSV/image replay, invalid NaN output, blackout recovery and no-tracking exit status')
