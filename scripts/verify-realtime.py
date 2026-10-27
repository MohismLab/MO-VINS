#!/usr/bin/env python3
"""Check the built RT implementation and module ABI; does not measure latency."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parent.parent
build = root / 'out/kernel'
manifest = json.loads((root / 'kernel/rt/manifest.json').read_text())
source_name = 'kernel-rt-src-' + manifest['patch_sha256'][:12]
assert (build / 'source').readlink().name == source_name, 'Kbuild source tree'
source = root / 'out' / source_name
config = dict(line.split('=', 1) for line in (build / '.config').read_text().splitlines()
              if line.startswith('CONFIG_') and '=' in line)
for key, value in {'PREEMPT_RT': 'y', 'PREEMPTION': 'y', 'PREEMPT_COUNT': 'y',
                   'HIGH_RES_TIMERS': 'y', 'HZ': '1000', 'IKCONFIG_PROC': 'y',
                   'DEBUG_ATOMIC_SLEEP': 'y', 'DEBUG_PREEMPT': 'y',
                   'USB_CONFIGFS_ACM': 'y', 'USB_CONFIGFS_F_UVC': 'y',
                   'VIDEO_ROCKCHIP_CIF': 'y', 'SPI_ROCKCHIP': 'y'}.items():
    assert config.get('CONFIG_' + key) == value, key
assert config.get('CONFIG_PREEMPT_NONE') != 'y'
assert config.get('CONFIG_FIQ_DEBUGGER') != 'y'
for name, hashes in manifest['files'].items():
    path = source / name
    actual = hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None
    assert actual == hashes['after'], f'RT source mismatch: {name}'
release = (build / 'include/config/kernel.release').read_text().strip()
assert release == '5.10.160-rt77-mohism', release
nm = root / 'vendor/luckfox-pico/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-nm'
symbols = subprocess.check_output([str(nm), str(build / 'vmlinux')], text=True)
for name in ['rt_spin_lock', 'rt_spin_unlock', 'rt_mutex_lock', 'preempt_schedule',
             'hrtimer_interrupt', 'irq_thread', 'irq_forced_thread_fn', 'printk_kthread_func']:
    assert any(line.endswith(' ' + name) for line in symbols.splitlines()), name
module = root / 'out/artifacts/mohism_imu.ko'
assert b'vermagic=' + release.encode() + b' ' in module.read_bytes(), 'IMU vermagic'
assert (root / 'out/artifacts/zImage').read_bytes() == (build / 'arch/arm/boot/zImage').read_bytes()
if '--rootfs' in sys.argv:
    target = root / 'out/buildroot/target'
    assert (target / 'usr/bin/cyclictest').is_file(), 'cyclictest missing'
    helper = Path('usr/bin/mohism-rt-check')
    assert (target / helper).read_bytes() == (root / 'board/mohism/rootfs-overlay' / helper).read_bytes()
    modules = target / 'lib/modules'
    assert [path.name for path in modules.iterdir() if path.is_dir()] == [release], 'stale kernel modules'
    assert (modules / release / 'extra/mohism_imu.ko').read_bytes() == module.read_bytes()
    assert (modules / release / 'modules.dep').is_file(), 'module dependencies missing'
report = {'kernel_release': release, 'patch_sha256': manifest['patch_sha256'],
          'preempt_rt': True, 'hz': 1000, 'board_boot_verified': False,
          'latency_measured': False}
output = root / 'out/validation/realtime-build.json'
output.parent.mkdir(exist_ok=True)
output.write_text(json.dumps(report, indent=2) + '\n')
print('PASS full RT source hashes, kernel configuration, linked RT symbols, IMU module ABI and kernel artifact')
print('Target boot and scheduling latency remain unmeasured.')
