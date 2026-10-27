#!/usr/bin/env python3
"""Exercise descriptor writes against a filesystem fixture, without USB emulation."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
helper = root / 'board/mohism/rootfs-overlay/usr/lib/mohism/uvc.sh'
profile = root / 'board/mohism/rootfs-overlay/usr/lib/mohism/performance.sh'
with tempfile.TemporaryDirectory() as temporary:
    base = Path(temporary)
    config = base / 'performance.conf'
    config.write_text('CAPTURE_FPS=120\nUSB_FPS=60\nDEPTH_FPS=30\n')
    # configfs creates these groups and default attributes on function creation.
    for kind in ['stereo','depth']:
        function = base / 'functions' / ('uvc.' + kind)
        for directory in ['streaming/class/fs','streaming/class/hs','control/class/fs','control/class/ss']:
            (function / directory).mkdir(parents=True)
    subprocess.run(['sh','-eu','-c', '''G=$1
. "$2"
. "$3"
create_uvc uvc.stereo 640 256000 2048 "$USB_INTERVAL" "$USB_BITRATE"
create_uvc uvc.depth 320 128000 1024 "$USB_DEPTH_INTERVAL" "$USB_DEPTH_BITRATE"
''','fixture',str(base),str(profile),str(helper)],
        env={**os.environ,'MOHISM_PERFORMANCE_CONFIG':str(config)},check=True)
    for kind,width,size,packet,interval,bitrate in [('stereo',640,256000,2048,166666,122880000),('depth',320,128000,1024,333333,30720000)]:
        function = base / 'functions' / ('uvc.' + kind)
        frame = function / f'streaming/uncompressed/u/{width}x200'
        for name,value in {'wWidth':width,'wHeight':200,'dwMaxVideoFrameBufferSize':size,
                           'dwDefaultFrameInterval':interval,'dwFrameInterval':interval,
                           'dwMinBitRate':bitrate,'dwMaxBitRate':bitrate}.items():
            assert int((frame / name).read_text()) == value, (kind,name)
        assert int((function/'streaming_maxpacket').read_text()) == packet
        assert (function/'streaming/class/hs/h').resolve() == function/'streaming/header/h'
        assert (function/'streaming/header/h/u').resolve() == function/'streaming/uncompressed/u'
    assert (base/'functions/uvc.depth/streaming/uncompressed/u/guidFormat').read_bytes() == b'Z16 '+bytes.fromhex('00001000800000aa00389b71')
startup=(root/'board/mohism/rootfs-overlay/etc/init.d/S40mohism-usb').read_text()
for instance,link in [('uvc.stereo','01-stereo'),('uvc.depth','02-depth'),('acm.usb0','03-acm'),('acm.usb1','04-debug')]:
    assert f'ln -s ../../functions/{instance} "$G/configs/c.1/{link}"' in startup
print('PASS dual UVC descriptor writes, Z16 GUID, geometry, rates and composite function order (offline fixture)')
