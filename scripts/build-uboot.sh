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
cp board/mohism/rv1106-mohism-uboot.dts "$U/arch/arm/dts/"
make -C "$U" luckfox_rv1106_uboot_defconfig rk-emmc.config
/work/vendor/luckfox-pico/sysdrv/source/kernel/scripts/config --file "$U/.config" --set-str DEFAULT_DEVICE_TREE rv1106-mohism-uboot
/work/vendor/luckfox-pico/sysdrv/source/kernel/scripts/config --file "$U/.config" --set-val SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR 0x440
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
