#!/usr/bin/env python3
"""Keep ACM0 binary-clean: only explicitly selected configfs ACM instances become consoles."""
from pathlib import Path
path = Path('vendor/luckfox-pico/sysdrv/source/kernel/drivers/usb/gadget/function/f_acm.c')
source = path.read_text()
original = 'ret = gserial_alloc_line(&opts->port_num);'
modified = 'ret = gserial_alloc_line_no_console(&opts->port_num); /* MOHISM: console is opt-in */'
if modified not in source:
    if source.count(original) != 1:
        raise RuntimeError('Pinned f_acm allocation site changed')
    path.write_text(source.replace(original, modified))
