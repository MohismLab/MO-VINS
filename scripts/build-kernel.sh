#!/bin/sh
set -eu
cd /work
sh scripts/prepare.sh
export ARCH=arm
export CROSS_COMPILE=/work/vendor/luckfox-pico/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-
mkdir -p out/bin out/artifacts
ln -sf /usr/bin/python3 out/bin/python
export PATH=/work/out/bin:$PATH
case "${1:-rt}" in
 rt)
  K=$(python3 scripts/prepare-rt.py)
  BUILD=/work/out/kernel
  ARTIFACTS=/work/out/artifacts
  MODULES=/work/out/modules
  ;;
 standard)
  K=/work/vendor/luckfox-pico/sysdrv/source/kernel
  BUILD=/work/out/kernel-standard
  ARTIFACTS=/work/out/artifacts/standard
  MODULES=/work/out/modules-standard
  ;;
 *) echo 'usage: build-kernel.sh [rt|standard]' >&2;exit 2;;
esac
# Kbuild cannot reuse an output directory tied to a different source tree.
if [ -L "$BUILD/source" ] && [ "$(readlink -f "$BUILD/source")" != "$K" ]; then
 mv "$BUILD" "$BUILD.previous-$(date -u +%Y%m%dT%H%M%S)"
fi
mkdir -p "$BUILD" "$ARTIFACTS"
make -C "$K" O="$BUILD" luckfox_rv1106_linux_defconfig
if [ "${1:-rt}" = rt ]; then
 "$K/scripts/kconfig/merge_config.sh" -m -O "$BUILD" "$BUILD/.config" board/mohism/kernel.config board/mohism/realtime.config
else
 "$K/scripts/kconfig/merge_config.sh" -m -O "$BUILD" "$BUILD/.config" board/mohism/kernel.config
fi
make -C "$K" O="$BUILD" olddefconfig
if [ "${1:-rt}" = rt ]; then
 grep -qx 'CONFIG_PREEMPT_RT=y' "$BUILD/.config"
 grep -qx 'CONFIG_HZ=1000' "$BUILD/.config"
fi
make -C "$K" O="$BUILD" -j"${JOBS:-8}" zImage modules rv1106g3-mohism-vins.dtb
# Keep external module outputs separate for the RT and standard kernels.
DRIVER=/work/out/kernel-driver-${1:-rt}
mkdir -p "$DRIVER"
cp kernel/Makefile kernel/mohism_imu.c include/mohism_imu.h "$DRIVER/"
make -C "$K" O="$BUILD" M="$DRIVER" -j"${JOBS:-8}" modules
cp "$BUILD/arch/arm/boot/zImage" "$BUILD/arch/arm/boot/dts/rv1106g3-mohism-vins.dtb" "$DRIVER/mohism_imu.ko" "$ARTIFACTS/"
make -C "$K" O="$BUILD" INSTALL_MOD_PATH="$MODULES" modules_install
make -C "$K" O="$BUILD" M="$DRIVER" INSTALL_MOD_PATH="$MODULES" modules_install
if [ "${1:-rt}" = rt ]; then
 cp "$DRIVER/mohism_imu.ko" kernel/mohism_imu.ko
 python3 scripts/verify-realtime.py
fi
