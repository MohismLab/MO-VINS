#!/usr/bin/env python3
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'host'))
import numpy as np
from mohism import unpack
rng = np.random.default_rng(17)
left = rng.integers(0,256,(200,320),dtype=np.uint8)
right = rng.integers(0,256,(200,320),dtype=np.uint8)
depth = rng.integers(0,65536,(200,320),dtype=np.uint16)
raw = np.full((200,1920),128,np.uint8)
raw[:,:640:2]=left
raw[:,640:1280:2]=right
raw[:,1280:]=depth.astype('<u2').view(np.uint8).reshape(200,640)
a,b,c=unpack(raw)
assert np.array_equal(a,left) and np.array_equal(b,right) and np.array_equal(c,depth)
try:
    unpack(raw[:1])
except ValueError:
    pass
else:
    raise AssertionError('short frame accepted')
print('PASS stereo tiles and all uint16 depth byte values preserved')
