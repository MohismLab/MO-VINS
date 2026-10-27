#!/usr/bin/env python3
"""Check requested timing coherence; no target performance claims."""
from pathlib import Path
import re
import os
import subprocess
import tempfile
from importlib.machinery import SourceFileLoader
root=Path(__file__).resolve().parent.parent
helper=root/'board/mohism/rootfs-overlay/usr/lib/mohism/performance.sh'
with tempfile.TemporaryDirectory() as work:
    path=Path(work)/'performance.conf'
    def run(capture,usb,depth):
        path.write_text(f'CAPTURE_FPS={capture}\nUSB_FPS={usb}\nDEPTH_FPS={depth}\n')
        return subprocess.run(['sh','-ec','. "$1"; printf "%s %s %s %s %s\\n" "$USB_INTERVAL" "$USB_BITRATE" "$FSIN_PERIOD" "$USB_DEPTH_INTERVAL" "$USB_DEPTH_BITRATE"', 'profile',str(helper)],
                              env={**os.environ,'MOHISM_PERFORMANCE_CONFIG':str(path)},capture_output=True,text=True)
    for capture in [15,30,60,120]:
        for usb in [15,30,60]:
            result=run(capture,usb,min(capture,30))
            if usb>capture:
                assert result.returncode!=0
            else:
                assert result.returncode==0, result.stderr
                assert list(map(int,result.stdout.split()))==[10000000//usb,256000*8*usb,1000000000//capture,10000000//min(usb,capture,30),128000*8*min(usb,capture,30)]
    for args in [(120,120,30),(0,30,30),(60,60,120),('abc',30,30)]:
        assert run(*args).returncode!=0, args
fdt=SourceFileLoader('artifact_check',str(root/'scripts/verify-artifacts.py')).load_module()
props=fdt.fdt_props((root/'out/artifacts/rv1106g3-mohism-vins.dtb').read_bytes())
highspeed=[key for key in props if key.endswith('/mohism,high-speed-readout')]
assert len(highspeed)==2, highspeed
assert all(key.replace('mohism,high-speed-readout','mohism,external-fsin') in props for key in highspeed)
print('PASS requested capture/USB/depth timing profiles, bandwidth arithmetic, rejection gates and compiled camera mode flags')

cache=(root/'out/buildroot/build/opencv4-4.6.0/buildroot-build/CMakeCache.txt').read_text()
assert 'ENABLE_FAST_MATH:BOOL=OFF' in cache
assert 'ENABLE_NEON:BOOL=ON' in cache
version=re.search(r'project\(mohism_vins VERSION ([0-9.]+)',(root/'CMakeLists.txt').read_text()).group(1)
flags=(root/f'out/buildroot/build/mohism-vins-{version}/CMakeFiles/mohism_core.dir/flags.make').read_text()
assert '-O3' in flags and '-fno-fast-math' in flags
print('PASS target OpenCV NEON / strict floating-point configuration and VIO release optimization flags')
