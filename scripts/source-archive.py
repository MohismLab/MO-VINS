#!/usr/bin/env python3
"""Bundle firmware corresponding source and pinned generated MAVLink headers, never private/build files."""
import io
import json
from pathlib import Path
import tarfile

root = Path(__file__).resolve().parent.parent
destination = root / 'out/artifacts/mohism-vins-source.tar.gz'
paths = ['CMakeLists.txt', 'README.md', 'TASKS.md', 'LICENSE', '.gitignore', 'sources.lock.json',
         'uboot', 'src', 'include', 'tests', 'host', 'scripts', 'board', 'buildroot', 'docker',
         'docs', 'hardware', 'qualification', 'third_party', 'kernel/Makefile', 'kernel/mohism_imu.c',
         'vendor/mavlink']

def clean(info):
    parts = Path(info.name).parts
    if '.git' in parts or '__pycache__' in parts or info.name.endswith('.pyc'):
        return None
    if not info.isfile() and not info.isdir():
        raise RuntimeError(f'Unexpected symlink or special file in source: {info.name}')
    info.uid = info.gid = 0
    info.uname = info.gname = 'root'
    return info

with tarfile.open(destination, 'w:gz') as archive:
    for item in paths:
        archive.add(root / item, arcname='mohism-vins/' + item, filter=clean)
    commit = json.loads((root / 'sources.lock.json').read_text())['mavlink']['commit']
    marker = (commit + '\n').encode()
    info = tarfile.TarInfo('mohism-vins/vendor/mavlink/.source-commit')
    info.size = len(marker); info.mode = 0o644
    archive.addfile(info, io.BytesIO(marker))
print(destination.name)
