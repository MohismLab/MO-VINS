#!/bin/sh
set -eu
cd /work
. ./scripts/env.sh
export ARCH=arm
export CROSS_COMPILE=/work/vendor/luckfox-pico/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-
U=/work/vendor/luckfox-pico/sysdrv/source/uboot/u-boot
mkdir -p out/uboot out/artifacts
# The SDK U-Boot packer expects an in-tree build and its sibling rkbin directory.
# U-Boot only uses UART2/SDMMC/USB/SARADC here; no stock camera/power GPIOs.
python3 scripts/patch-uboot-console.py
cp board/mohism/rv1106-mohism-uboot.dts "$U/arch/arm/dts/"
make -C "$U" luckfox_rv1106_uboot_defconfig rk-emmc.config
/work/vendor/luckfox-pico/sysdrv/source/kernel/scripts/config --file "$U/.config" --set-str DEFAULT_DEVICE_TREE rv1106-mohism-uboot
/work/vendor/luckfox-pico/sysdrv/source/kernel/scripts/config --file "$U/.config" --set-val SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR 0x440
CFG=/work/vendor/luckfox-pico/sysdrv/source/kernel/scripts/config
for symbol in DM_REGULATOR DM_REGULATOR_FIXED DM_REGULATOR_GPIO USB DM_USB DM_USB_GADGET USB_GADGET USB_GADGET_DOWNLOAD USB_DWC3 USB_DWC3_GADGET USB_DWC3_GENERIC PHY PHY_ROCKCHIP_INNO_USB2 MISC CONSOLE_MUX SYS_CONSOLE_IS_IN_ENV MOHISM_USB_CONSOLE; do
 "$CFG" --file "$U/.config" --enable "$symbol"
done
for symbol in FASTBOOT CMD_FASTBOOT USB_FUNCTION_FASTBOOT SPL_DM_USB SPL_DM_USB_GADGET USB_XHCI_HCD USB_STORAGE USB_DWC3_HOST; do
 "$CFG" --file "$U/.config" --disable "$symbol"
done
"$CFG" --file "$U/.config" --set-val BOOTDELAY 5 --set-str USB_GADGET_MANUFACTURER MOHISM --set-val USB_GADGET_VENDOR_NUM 0x1d6b --set-val USB_GADGET_PRODUCT_NUM 0x0105
make -C "$U" olddefconfig
make -C "$U" CROSS_COMPILE="$CROSS_COMPILE" PYTHON=python3 -j"${JOBS:-8}" all
cd "$U"
# SDK script hard-codes python2; its selected RV1106 build works with Python 3.
sed -i 's/make PYTHON=python2 /make PYTHON=python3 /' make.sh
./make.sh --spl-new CROSS_COMPILE="$CROSS_COMPILE"
cp uboot.img /work/out/artifacts/
for image in rv1106*download*.bin rv1106*idblock*.img; do
 [ ! -f "$image" ] || cp "$image" /work/out/artifacts/
done
cp .config /work/out/uboot/config
