#!/usr/bin/env python3
"""Select the SDK's existing 120fps readout mode for external-trigger cameras.
Do not invent a sensor register table. FSIN determines the requested cadence.
"""
from pathlib import Path
p=Path('vendor/luckfox-pico/sysdrv/source/kernel/drivers/media/i2c/ov9281.c')
s=p.read_text()
def replace(old,new):
    global s
    if new in s:return
    if s.count(old)!=1:raise RuntimeError('Pinned OV9281 mode-selection site changed')
    s=s.replace(old,new)
replace('ov9281_find_best_fit(struct v4l2_subdev_format *fmt)',
        'ov9281_find_best_fit(struct ov9281 *ov9281, struct v4l2_subdev_format *fmt)')
replace('int cur_best_fit = 0;', 'int cur_best_fit = device_property_read_bool(&ov9281->client->dev, "mohism,high-speed-readout") ? 1 : 0;')
replace('dist = ov9281_get_reso_dist(&supported_modes[i], framefmt);', '''if (device_property_read_bool(&ov9281->client->dev, "mohism,high-speed-readout") && i != 1)
			continue;
		dist = ov9281_get_reso_dist(&supported_modes[i], framefmt);''')
replace('mode = ov9281_find_best_fit(fmt);', 'mode = ov9281_find_best_fit(ov9281, fmt);')
replace('ov9281->cur_mode = &supported_modes[0];', '''ov9281->cur_mode = &supported_modes[
		device_property_read_bool(dev, "mohism,high-speed-readout") ? 1 : 0];''')
p.write_text(s)
