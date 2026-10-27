#!/usr/bin/env python3
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'host'))
import numpy as np
from mohism import unpack_stereo, unpack_depth
rng = np.random.default_rng(17)
left = rng.integers(0,256,(200,320),dtype=np.uint8)
right = rng.integers(0,256,(200,320),dtype=np.uint8)
depth = ((np.arange(64000, dtype=np.uint32)+1536)%65536).astype(np.uint16).reshape(200,320)
raw = np.full((200,1280),128,np.uint8)
raw[:,:640:2]=left
raw[:,640::2]=right
a,b=unpack_stereo(raw)
c=unpack_depth(depth.astype('<u2').view(np.uint8))
assert np.array_equal(a,left) and np.array_equal(b,right) and np.array_equal(c,depth)
assert np.array_equal(unpack_depth(depth),depth)
for decode, invalid in [(unpack_stereo,raw[:1]),(unpack_depth,raw),(unpack_stereo,depth)]:
    try:
        decode(invalid)
    except ValueError:
        pass
    else:
        raise AssertionError('invalid frame accepted')
# Include values omitted by the first ramp, including zero and uint16 maximum.
other = (np.arange(64000, dtype=np.uint32)%65536).astype(np.uint16).reshape(200,320)
assert np.array_equal(unpack_depth(other.view(np.uint8)), other)
print('PASS separate stereo and Z16 depth decoders, all uint16 values preserved')
