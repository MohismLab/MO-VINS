#!/bin/sh
# Run numerical and map tests on ARM userspace under QEMU, without camera hardware.
set -eu
cd /work
sh scripts/prepare.sh
cmake -S out/dist -B out/arm-validation -G Ninja \
 -DCMAKE_TOOLCHAIN_FILE=/work/out/buildroot/host/share/buildroot/toolchainfile.cmake \
 -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build out/arm-validation --target tight-test loop-test initialization-test -j4
for executable in tight-test loop-test initialization-test; do
 qemu-arm -cpu cortex-a7 -L /work/out/buildroot/target "/work/out/arm-validation/$executable"
done
