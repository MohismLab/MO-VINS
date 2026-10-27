#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
. ./scripts/env.sh
ROOT=$(pwd)
SDK=$ROOT/vendor/luckfox-pico
test -f "$SDK/sysdrv/source/kernel/Makefile"
mkdir -p out/dist/vendor
cp CMakeLists.txt out/dist/
cp -a src include tests out/dist/
cp LICENSE out/dist/
cp -a third_party host out/dist/
rm -rf out/dist/vendor/mavlink
cp -a vendor/mavlink out/dist/vendor/mavlink
rm -rf out/dist/vendor/mavlink/.git
cp board/mohism/rv1106g3-mohism-vins.dts "$SDK/sysdrv/source/kernel/arch/arm/boot/dts/"
python3 scripts/patch_ov9281.py
chmod +x board/mohism/rootfs-overlay/etc/init.d/S40mohism-usb board/mohism/rootfs-overlay/etc/init.d/S50mohism-vins
