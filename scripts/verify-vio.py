#!/usr/bin/env python3
"""Verify the packaged estimator and its complete ARM shared-library closure."""
from pathlib import Path
import re
import subprocess

root = Path('out/buildroot/target')
app = root / 'usr/bin/mohism-vins'
reader = next(Path('out/buildroot/host/bin').glob('*-readelf'))
header = subprocess.check_output([str(reader), '-h', str(app)], text=True)
assert 'ARM' in header and 'ELF32' in header
pending, checked = [app], set()
dependencies = set()
while pending:
    binary = pending.pop()
    if binary in checked:
        continue
    checked.add(binary)
    info = subprocess.check_output([str(reader), '-d', str(binary)], text=True)
    for name in re.findall(r'\(NEEDED\).*?\[(.*?)\]', info):
        dependencies.add(name)
        candidates = [root / folder / name for folder in ('lib', 'usr/lib')]
        candidate = next((path for path in candidates if path.is_file()), None)
        assert candidate, f'missing target dependency {name} needed by {binary}'
        pending.append(candidate)
assert any(name.startswith('libceres.so') for name in dependencies)
assert not any(name.startswith('libmohism_core.so') for name in dependencies)
payload = app.read_bytes()
for marker in (b'joint visual-inertial optimization failed', b'joint_marginalizations',
               b'tracking: tightly coupled stereo/IMU', b'map rejected:'):
    assert marker in payload, marker
for license_file in ('mohism-vins/LICENSE', 'mohism-vins/VINS-Fusion-NOTICE.md', 'ceres-solver/LICENSE'):
    assert (root / 'usr/share/licenses' / license_file).is_file(), license_file
config = (root / 'etc/mohism/calibration.yml').read_text()
for setting in ('window_size: 6', 'estimate_extrinsics: 0', 'estimate_time_offset: 0', 'loop_closure: 1'):
    assert setting in config, setting
assert (Path('out/artifacts/mohism-vins').read_bytes() == payload)
print(f'PASS ARM estimator, Ceres and {len(checked)-1} shared objects, joint/map diagnostics and licenses')
