#!/usr/bin/env python3
"""Calibrate pinhole stereo from raw-preview PNG pairs (calibrated: 0).
Does not invent camera/IMU extrinsics or automatically enable VIO.
"""
import argparse
from pathlib import Path
import cv2
import numpy as np

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--columns', type=int, required=True, help='chessboard inner corners')
    parser.add_argument('--rows', type=int, required=True)
    parser.add_argument('--square-metres', type=float, required=True)
    parser.add_argument('--output', type=Path, default=Path('stereo-measured.yml'))
    args = parser.parse_args()
    if args.columns < 3 or args.rows < 3 or args.square_metres <= 0:
        raise ValueError('Invalid chessboard geometry')
    pattern = (args.columns, args.rows)
    grid = np.zeros((args.columns*args.rows,3),np.float32)
    grid[:,:2] = np.mgrid[0:args.columns,0:args.rows].T.reshape(-1,2)*args.square_metres
    objects, left, right = [], [], []
    size = None
    for file in sorted(args.directory.glob('*-left.png')):
        other = file.with_name(file.name.replace('-left.png','-right.png'))
        a,b=cv2.imread(str(file),0),cv2.imread(str(other),0)
        if a is None or b is None or a.shape != b.shape:
            raise ValueError(f'Invalid stereo pair: {file}')
        if size is not None and size != a.shape[::-1]:
            raise ValueError('Mixed resolutions')
        size=a.shape[::-1]
        ok1,p1=cv2.findChessboardCornersSB(a,pattern)
        ok2,p2=cv2.findChessboardCornersSB(b,pattern)
        if ok1 and ok2:
            objects.append(grid.copy());left.append(p1);right.append(p2)
    if len(objects)<15:
        raise ValueError(f'Need at least 15 diverse detected pairs; got {len(objects)}')
    rms1,k1,d1,_,_=cv2.calibrateCamera(objects,left,size,None,None)
    rms2,k2,d2,_,_=cv2.calibrateCamera(objects,right,size,None,None)
    rms,k1,d1,k2,d2,R,T,_,_=cv2.stereoCalibrate(objects,left,right,k1,d1,k2,d2,size,flags=cv2.CALIB_FIX_INTRINSIC)
    print(f'pairs={len(objects)} left RMS={rms1:.4f}px right RMS={rms2:.4f}px stereo RMS={rms:.4f}px baseline={np.linalg.norm(T):.6f}m')
    if rms>1 or rms1>1 or rms2>1:
        raise ValueError('Reprojection RMS >1px: collect better views and check synchronization')
    # The firmware resizes to this preview size before rectification. Store equivalent
    # native K so the runtime scale reproduces the measured preview K exactly.
    k1[0,:]*=1280/size[0];k1[1,:]*=800/size[1]
    k2[0,:]*=1280/size[0];k2[1,:]*=800/size[1]
    fs=cv2.FileStorage(str(args.output),cv2.FILE_STORAGE_WRITE)
    fs.write('calibrated',0)
    for key,value in dict(K1=k1,D1=d1,K2=k2,D2=d2,R=R,T=T).items():
        fs.write(key,value)
    fs.release()
    print(f'Stereo measurements saved to {args.output}; merge into calibration template and independently calibrate Rbc/tbc/Rbi/imu_offset.')

if __name__=='__main__':
    main()
