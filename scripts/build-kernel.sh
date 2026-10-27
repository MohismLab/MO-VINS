#!/bin/sh
set -eu
cd /work
sh scripts/prepare.sh
export ARCH=arm
export CROSS_COMPILE=/work/vendor/luckfox-pico/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-
mkdir -p out/bin out/kernel out/artifacts
ln -sf /usr/bin/python3 out/bin/python
export PATH=/work/out/bin:$PATH
K=/work/vendor/luckfox-pico/sysdrv/source/kernel
make -C "$K" O=/work/out/kernel luckfox_rv1106_linux_defconfig
"$K/scripts/kconfig/merge_config.sh" -m -O out/kernel out/kernel/.config board/mohism/kernel.config
make -C "$K" O=/work/out/kernel olddefconfig
make -C "$K" O=/work/out/kernel -j"${JOBS:-8}" zImage modules rv1106g3-mohism-vins.dtb
make -C "$K" O=/work/out/kernel M=/work/kernel -j"${JOBS:-8}" modules
cp out/kernel/arch/arm/boot/zImage out/kernel/arch/arm/boot/dts/rv1106g3-mohism-vins.dtb kernel/mohism_imu.ko out/artifacts/
make -C "$K" O=/work/out/kernel INSTALL_MOD_PATH=/work/out/modules modules_install
make -C "$K" O=/work/out/kernel M=/work/kernel INSTALL_MOD_PATH=/work/out/modules modules_install
