#!/bin/sh
set -eu
TARGET_DIR=$1
ROOT=$(CDPATH='' cd -- "$(dirname "$0")/.." && pwd)
RELEASE=$(cat "$ROOT/out/kernel/include/config/kernel.release")
test -f "$ROOT/kernel/mohism_imu.ko"
mkdir -p "$TARGET_DIR/lib/modules/$RELEASE/extra"
cp -a "$ROOT/out/modules/lib/modules/$RELEASE/." "$TARGET_DIR/lib/modules/$RELEASE/"
cp "$ROOT/kernel/mohism_imu.ko" "$TARGET_DIR/lib/modules/$RELEASE/extra/"
rm -f "$TARGET_DIR/lib/modules/$RELEASE/build" "$TARGET_DIR/lib/modules/$RELEASE/source"
"$HOST_DIR/sbin/depmod" -b "$TARGET_DIR" "$RELEASE"
mkdir -p "$TARGET_DIR/userdata"
mkdir -p "$TARGET_DIR/usr/share/licenses/mohism-vins"
cp "$ROOT/LICENSE" "$TARGET_DIR/usr/share/licenses/mohism-vins/"
cp "$ROOT/third_party/VINS-Fusion/NOTICE.md" "$TARGET_DIR/usr/share/licenses/mohism-vins/VINS-Fusion-NOTICE.md"
if ! grep -q '^ttyGS1::respawn:' "$TARGET_DIR/etc/inittab"; then
 printf '\nttyGS1::respawn:/sbin/getty -L ttyGS1 115200 vt100\n' >> "$TARGET_DIR/etc/inittab"
fi
if ! grep -qx ttyGS1 "$TARGET_DIR/etc/securetty"; then
 printf 'ttyGS1\n' >> "$TARGET_DIR/etc/securetty"
fi
sed -i 's|^/dev/root[[:space:]]*/[[:space:]]*ext2|/dev/root / ext4|' "$TARGET_DIR/etc/fstab"
if ! grep -q '^/dev/mmcblk1p5' "$TARGET_DIR/etc/fstab"; then
 printf '/dev/mmcblk1p5 /userdata ext4 defaults,noatime 0 0\n' >> "$TARGET_DIR/etc/fstab"
fi
