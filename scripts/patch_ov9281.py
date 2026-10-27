#!/usr/bin/env python3
"""Apply the small external FSIN mode to the pinned SDK, idempotently.
Register pair from Antmicro's OV9281 FSYNC_SLAVE table. Requires board validation.
"""
from pathlib import Path
p = Path('vendor/luckfox-pico/sysdrv/source/kernel/drivers/media/i2c/ov9281.c')
s = p.read_text()
marker = '/* MOHISM external FSIN mode */'
if marker not in s:
    start = s.index('static int __ov9281_start_stream(')
    needle = '\n\treturn ov9281_write_reg(ov9281->client, OV9281_REG_CTRL_MODE,'
    where = s.index(needle, start)
    addition = '''
\t/* MOHISM external FSIN mode */
\tif (device_property_read_bool(&ov9281->client->dev, "mohism,external-fsin")) {
\t\tret = ov9281_write_reg(ov9281->client, 0x3006,
\t\t\tOV9281_REG_VALUE_08BIT, 0x00);
\t\tif (ret)
\t\t\treturn ret;
\t\tret = ov9281_write_reg(ov9281->client, 0x3823,
\t\t\tOV9281_REG_VALUE_08BIT, 0x30);
\t\tif (ret)
\t\t\treturn ret;
\t}
'''
    s = s[:where] + addition + s[where:]
    s = s.replace('#include <linux/i2c.h>', '#include <linux/i2c.h>\n#include <linux/property.h>', 1)
    p.write_text(s)

# The payload contains raw uint16 depth bytes. Do not advertise image-processing
# controls that the service cannot implement without corrupting that payload.
p = Path('vendor/luckfox-pico/sysdrv/source/kernel/drivers/usb/gadget/function/f_uvc.c')
s = p.read_text()
s = s.replace('cd->bmControls[0]\t\t= 2;', 'cd->bmControls[0]\t\t= 0; /* MOHISM raw payload */')
s = s.replace('pd->bmControls[0]\t\t= 1;', 'pd->bmControls[0]\t\t= 0; /* MOHISM raw payload */')
p.write_text(s)
