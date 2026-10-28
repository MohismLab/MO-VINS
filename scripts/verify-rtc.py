#!/usr/bin/env python3
"""Verify built-in RTC support and the compiled board wiring."""
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parent.parent
build = Path(sys.argv[1]) if len(sys.argv) > 1 else root / 'out/kernel'
config = (build / '.config').read_text().splitlines()
for setting in ('I2C=y', 'I2C_RK3X=y', 'RTC_CLASS=y', 'RTC_INTF_DEV=y',
                'RTC_HCTOSYS=y', 'RTC_HCTOSYS_DEVICE="rtc0"', 'RTC_DRV_PCF8563=y'):
    assert 'CONFIG_' + setting in config, setting
dtb = build / 'arch/arm/boot/dts/rv1106g3-mohism-vins.dtb'
# Follow DT paths using fdtget rather than depending on decompiler formatting.
fdtget = root / 'out/buildroot/host/bin/fdtget'
if not fdtget.exists():
    fdtget = Path('fdtget')
def get(path, prop, kind='s'):
    return subprocess.check_output([str(fdtget), '-t', kind, str(dtb), path, prop], text=True).strip()
rtc = get('/aliases', 'rtc0')
assert get(rtc, 'compatible') == 'nxp,pcf8563'
assert get(rtc, 'reg', 'x') == '51'
bus = rtc.rsplit('/', 1)[0]
assert get(bus, 'status') == 'okay'
assert get(bus, 'clock-frequency', 'u') == '100000'
pins = get('/pinctrl/i2c1/i2c1m0-xfer', 'rockchip,pins', 'u').split()
assert pins[:3] == ['0', '5', '1'] and pins[4:7] == ['0', '6', '1'], pins
assert get('/rtc@ff1c0000', 'status') == 'disabled'
assert 'pcf8563_rtc_read_time' in (build / 'System.map').read_text()
print('PASS built-in RTC driver, rtc0 alias, I2C1 M0 pins, address and clock rate')
