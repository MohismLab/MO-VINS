#!/usr/bin/env python3
"""Check built USB console integration; does not emulate or validate a physical USB bus."""
from pathlib import Path
import subprocess
from importlib.machinery import SourceFileLoader

root = Path(__file__).resolve().parent.parent
k = root / 'out/kernel'
u = root / 'vendor/luckfox-pico/sysdrv/source/uboot/u-boot'
target = root / 'out/buildroot/target'
def config(path):
    return dict(line.split('=', 1) for line in path.read_text().splitlines()
                if line.startswith('CONFIG_') and '=' in line)
kconf, uconf = config(k / '.config'), config(root / 'out/uboot/config')
for name in ['USB_CONFIGFS_ACM', 'USB_CONFIGFS_F_UVC', 'U_SERIAL_CONSOLE']:
    assert kconf['CONFIG_' + name] == 'y', name
for name in ['MOHISM_USB_CONSOLE', 'CONSOLE_MUX', 'SYS_CONSOLE_IS_IN_ENV',
             'DM_USB_GADGET', 'DM_DEVICE_REMOVE', 'USB_DWC3_GADGET']:
    assert uconf['CONFIG_' + name] == 'y', name
assert uconf['CONFIG_BOOTDELAY'] == '5'
assert uconf['CONFIG_USB_GADGET_PRODUCT_NUM'] == '0x0105'
nm = root / 'vendor/luckfox-pico/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-nm'
symbols = subprocess.check_output([str(nm), str(u / 'u-boot')], text=True)
for name in ['mohism_usb_console_init', 'mohism_usb_console_stop',
             'dm_usb_gadget_handle_interrupts', 'configure']:
    assert any(line.endswith(' ' + name) for line in symbols.splitlines()), name
# Read the compiled DTB rather than trusting DTS source text.
fdt = SourceFileLoader('artifact_check', str(root / 'scripts/verify-artifacts.py')).load_module()
bootargs = fdt.fdt_props((root / 'out/artifacts/rv1106g3-mohism-vins.dtb').read_bytes())['/chosen/bootargs']
assert b'console=ttyGS1' in bootargs and b'console=ttyGS0' not in bootargs
assert b'console=ttyS2' in bootargs
inittab = (target / 'etc/inittab').read_text()
assert inittab.count('ttyGS1::respawn:/sbin/getty -L ttyGS1 115200 vt100') == 1
assert 'ttyGS0::' not in inittab
assert 'ttyGS1' in (target / 'etc/securetty').read_text().splitlines()
assert (target / 'etc/init.d/S40mohism-usb').read_bytes() == (root / 'board/mohism/rootfs-overlay/etc/init.d/S40mohism-usb').read_bytes()
# This allocation choice is essential: enabling U_SERIAL_CONSOLE must never auto-register MAVLink.
acm = (root / 'vendor/luckfox-pico/sysdrv/source/kernel/drivers/usb/gadget/function/f_acm.c').read_text()
assert 'ret = gserial_alloc_line_no_console(&opts->port_num);' in acm
assert 'ret = gserial_alloc_line(&opts->port_num);' not in acm
assert (root / 'out/artifacts/uboot.img').stat().st_size <= 256 * 1024
print('PASS built kernel/U-Boot USB console configuration, ELF symbols, DTB, getty, ACM isolation and partition fit')
