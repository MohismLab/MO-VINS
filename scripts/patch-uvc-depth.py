#!/usr/bin/env python3
"""Enable the standard Z16 uncompressed format in the pinned gadget driver."""
from pathlib import Path
p = Path('vendor/luckfox-pico/sysdrv/source/kernel/drivers/usb/gadget/function/uvc_v4l2.c')
s = p.read_text()
if 'V4L2_PIX_FMT_Z16' not in s:
    needle = '{ 16, V4L2_PIX_FMT_YUYV  },'
    if needle not in s:
        raise SystemExit('Unexpected UVC format table; review Z16 port')
    s = s.replace(needle, needle + '\n\t{ 16, V4L2_PIX_FMT_Z16   },', 1)
    p.write_text(s)
