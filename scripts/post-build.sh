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
sed -i 's|^/dev/root[[:space:]]*/[[:space:]]*ext2|/dev/root / ext4|' "$TARGET_DIR/etc/fstab"
if ! grep -q '^/dev/mmcblk1p5' "$TARGET_DIR/etc/fstab"; then
 printf '/dev/mmcblk1p5 /userdata ext4 defaults,noatime 0 0\n' >> "$TARGET_DIR/etc/fstab"
fi
