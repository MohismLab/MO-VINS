#!/usr/bin/env python3
"""Apply the board CDC console integration to the pinned SDK, idempotently."""
from pathlib import Path
import shutil
root = Path('vendor/luckfox-pico/sysdrv/source/uboot/u-boot')
def patch(name, old, new):
    p = root / name
    text = p.read_text()
    if new in text:
        return
    if text.count(old) != 1:
        raise RuntimeError(f'Pinned U-Boot integration site changed: {name}')
    p.write_text(text.replace(old, new))
shutil.copyfile('uboot/mohism_usb_console.c', root / 'drivers/usb/gadget/mohism_usb_console.c')
patch('drivers/usb/gadget/Kconfig', '# USB Gadget support on a system involves', '''# USB Gadget support on a system involves

config MOHISM_USB_CONSOLE
	bool "MOHISM CDC ACM boot console"
	depends on USB_GADGET_DOWNLOAD && CONSOLE_MUX && DM_USB_GADGET
''')
patch('drivers/usb/gadget/Makefile', 'obj-$(CONFIG_USB_GADGET_DOWNLOAD) += g_dnl.o', 'obj-$(CONFIG_USB_GADGET_DOWNLOAD) += g_dnl.o\nobj-$(CONFIG_MOHISM_USB_CONSOLE) += mohism_usb_console.o')
patch('common/stdio.c', '\tserial_stdio_init ();', '''\tserial_stdio_init ();
#if defined(CONFIG_MOHISM_USB_CONSOLE) && !defined(CONFIG_SPL_BUILD)
	{ extern int mohism_usb_console_init(void); mohism_usb_console_init(); }
#endif''')
patch('arch/arm/lib/bootm.c', '\tboard_quiesce_devices(images);', '''#if defined(CONFIG_MOHISM_USB_CONSOLE) && !defined(CONFIG_SPL_BUILD)
	{ extern void mohism_usb_console_stop(void); mohism_usb_console_stop(); }
#endif
	board_quiesce_devices(images);''')
patch('drivers/usb/gadget/g_dnl.c', 'static const char product[] = "USB download gadget";', '''#ifdef CONFIG_MOHISM_USB_CONSOLE
static const char product[] = "MOHISM U-Boot Debug";
#else
static const char product[] = "USB download gadget";
#endif''')
patch('board/rockchip/evb_rv1106/evb_rv1106.c', '#ifdef CONFIG_USB_DWC3', '#if defined(CONFIG_USB_DWC3) && !defined(CONFIG_DM_USB_GADGET)')
patch('drivers/usb/dwc3/dwc3-generic.c', '#if defined(CONFIG_SPL_USB_HOST_SUPPORT) || !defined(CONFIG_SPL_BUILD)', '#if defined(CONFIG_SPL_USB_HOST_SUPPORT) || (!defined(CONFIG_SPL_BUILD) && defined(CONFIG_USB_XHCI_HCD))')
