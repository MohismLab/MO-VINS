#!/usr/bin/env python3
"""Verify image layout/CRC and exact embedded payloads without mounting anything."""
import hashlib
import json
from pathlib import Path
import struct
import zlib

def equal_region(file, offset, payload):
    with file.open('rb') as stream, payload.open('rb') as expected:
        stream.seek(offset)
        while True:
            chunk = expected.read(1024*1024)
            if not chunk:
                return
            if stream.read(len(chunk)) != chunk:
                raise AssertionError(f'Embedded payload mismatch: {payload}')

def fdt_props(data):
    magic,total,off_struct,off_strings,_,_,_,_,size_strings,size_struct=struct.unpack_from('>10I',data)
    assert magic == 0xd00dfeed and total <= len(data)
    strings=data[off_strings:off_strings+size_strings]
    pos=off_struct
    stack=[]
    props={}
    while pos<off_struct+size_struct:
        token=struct.unpack_from('>I',data,pos)[0];pos+=4
        if token==1:
            end=data.index(b'\0',pos)
            stack.append(data[pos:end].decode());pos=(end+4)&~3
        elif token==2:
            stack.pop()
        elif token==3:
            length,nameoff=struct.unpack_from('>II',data,pos);pos+=8
            name=strings[nameoff:strings.index(b'\0',nameoff)].decode()
            props['/'.join(stack)+'/'+name]=data[pos:pos+length];pos=(pos+length+3)&~3
        elif token==4:
            continue
        elif token==9:
            break
        else:
            raise AssertionError(f'Unknown FDT token {token}')
    return props

def main():
    directory=Path('out/artifacts')
    env=(directory/'env.img').read_bytes()
    assert zlib.crc32(env[4:]) == int.from_bytes(env[:4],'little'), 'ENV CRC'
    assert b'root=/dev/mmcblk1p6' in env
    for entry in json.loads((directory/'layout.json').read_text()):
        file=directory/entry['file']
        assert file.stat().st_size <= entry['size_bytes']
        equal_region(directory/'mohism-sd.img',entry['offset_bytes'],file)
    props=fdt_props((directory/'boot.img').read_bytes())
    for image, filename in [('kernel','zImage'),('fdt','rv1106g3-mohism-vins.dtb')]:
        payload=props[f'/images/{image}/data']
        assert payload == (directory/filename).read_bytes(), f'FIT {image} payload'
        assert hashlib.sha256(payload).digest() == props[f'/images/{image}/hash/value'], f'FIT {image} hash'
    dt=fdt_props((directory/'rv1106g3-mohism-vins.dtb').read_bytes())
    assert b'/dev/mmcblk1p6' in dt['/chosen/bootargs']
    assert b'console=ttyGS1' in dt['/chosen/bootargs']
    assert b'console=ttyGS0' not in dt['/chosen/bootargs']
    assert dt['/spi@ff500000/imu@0/compatible'].rstrip(b'\0') == b'mohism,lsm6dso-imu'
    print('PASS SD payload offsets, ENV CRC, FIT kernel/DTB SHA256, root partition and IMU binding')

if __name__=='__main__':
    main()
