#!/usr/bin/env python3
"""Assemble the SDK ENV partition format, never open a block device."""
from pathlib import Path
import json
import shutil

directory = Path('/work/out/artifacts')
kb = 1024
mb = kb * kb
idblocks = list(directory.glob('rv1106*idblock*.img'))
if len(idblocks) != 1:
    raise RuntimeError('Expected exactly one pinned RV1106 idblock')
layout = [
    ('env', 'env.img', 0, 32*kb),
    ('idblock', idblocks[0].name, 32*kb, 512*kb),
    ('uboot', 'uboot.img', 544*kb, 256*kb),
    ('boot', 'boot.img', 800*kb, 32*mb),
    ('userdata', 'userdata.img', 800*kb+32*mb, 16*mb),
    ('rootfs', 'rootfs.img', 800*kb+48*mb, 256*mb),
]
image = directory / 'mohism-sd.img'
with image.open('wb') as output:
    output.truncate(800*kb+304*mb)
    for index, (name, filename, offset, maximum) in enumerate(layout,1):
        path = directory / filename
        if path.stat().st_size > maximum:
            raise RuntimeError(f'{name} exceeds its partition')
        output.seek(offset)
        with path.open('rb') as source:
            shutil.copyfileobj(source, output)
        print(f'p{index}: {name:8} byte={offset:10} sector={offset//512:8} max={maximum}')
(directory/'layout.json').write_text(json.dumps([
    dict(partition=i,name=n,file=f,offset_bytes=o,size_bytes=s)
    for i,(n,f,o,s) in enumerate(layout,1)
], indent=2)+'\n')
