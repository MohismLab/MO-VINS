#!/usr/bin/env python3
"""Apply the reviewed ARM RT port to an isolated, hash-checked SDK copy."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
sdk = root / 'vendor/luckfox-pico/sysdrv/source/kernel'
port = root / 'kernel/rt'
manifest = json.loads((port / 'manifest.json').read_text())
patch = port / 'preempt-rt-arm.patch'

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None

def verify(directory, phase):
    for name, hashes in manifest['files'].items():
        if digest(directory / name) != hashes[phase]:
            raise SystemExit(f'RT {phase} hash mismatch: {directory / name}')

if digest(patch) != manifest['patch_sha256']:
    raise SystemExit('RT patch checksum mismatch')
marker = root / 'vendor/luckfox-pico/.source-commit'
if marker.exists():
    actual = marker.read_text().strip()
else:
    actual = subprocess.check_output(['git', '-C', str(sdk), 'rev-parse', 'HEAD'], text=True).strip()
if actual != manifest['sdk_commit']:
    raise SystemExit('RT port requires the pinned SDK commit')
verify(sdk, 'before')
output = root / 'out'
output.mkdir(exist_ok=True)
target = output / ('kernel-rt-src-' + manifest['patch_sha256'][:12])
if not target.exists():
    temporary = Path(tempfile.mkdtemp(prefix='kernel-rt-preparing-', dir=output))
    try:
        subprocess.run(['cp', '-a', '--reflink=auto', str(sdk) + '/.', str(temporary)], check=True)
        log = output / 'validation/rt-patch.log'
        log.parent.mkdir(exist_ok=True)
        with log.open('w') as stream:
            subprocess.run(['patch', '--batch', '--forward', '--fuzz=0', '-p1',
                            '-d', str(temporary), '-i', str(patch)],
                           stdout=stream, stderr=subprocess.STDOUT, check=True)
        verify(temporary, 'after')
        (temporary / '.mohism-rt.json').write_text(json.dumps({
            'sdk_commit': actual, 'patch_sha256': manifest['patch_sha256']}) + '\n')
        temporary.rename(target)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
verify(target, 'after')
# Board wiring and camera mode changes must also reach an already prepared tree.
shutil.copyfile(root / 'board/mohism/rv1106g3-mohism-vins.dts',
                target / 'arch/arm/boot/dts/rv1106g3-mohism-vins.dts')
# Project gadget patches are outside the RT manifest and can change independently.
for name in ['f_uvc.c', 'f_acm.c', 'uvc_v4l2.c']:
    relative = Path('drivers/usb/gadget/function') / name
    shutil.copyfile(sdk / relative, target / relative)
print(target)
