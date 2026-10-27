#!/bin/sh
set -eu
cd /work
test -f out/buildroot/images/rootfs.ext4
test -f out/artifacts/uboot.img
test -f out/artifacts/mohism_imu.ko
cp -L out/buildroot/images/rootfs.ext4 out/artifacts/rootfs.img
cp board/mohism/boot.its out/artifacts/boot.its
cp out/buildroot/target/usr/bin/mohism-vins out/artifacts/
cd out/artifacts
/work/vendor/luckfox-pico/sysdrv/source/uboot/u-boot/tools/mkimage -f boot.its boot.img
cat > env.txt <<'EOF'
blkdevparts=mmcblk1:32K(env),512K@32K(idblock),256K(uboot),32M(boot),16M(userdata),256M(rootfs)
sys_bootargs=root=/dev/mmcblk1p6 rootwait rootfstype=ext4
bootcmd=boot_fit;
EOF
/work/vendor/luckfox-pico/sysdrv/source/uboot/u-boot/tools/mkenvimage -s 32768 -p 0x0 -o env.img env.txt
truncate -s 16M userdata.img
/work/out/buildroot/host/sbin/mkfs.ext4 -F -L userdata userdata.img
python3 /work/scripts/make-sd-image.py
cd /work
python3 scripts/verify-artifacts.py
python3 scripts/verify-usb-debug.py
python3 scripts/verify-performance.py
python3 scripts/verify-realtime.py --rootfs
python3 scripts/source-archive.py
cd out/artifacts
sha256sum mohism-sd.img env.img uboot.img boot.img rootfs.img userdata.img mohism-vins mohism-vins-source.tar.gz mohism_imu.ko rv1106g3-mohism-vins.dtb rv1106*idblock*.img rv1106*download*.bin > SHA256SUMS
