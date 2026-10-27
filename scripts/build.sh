#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
. ./scripts/env.sh
export http_proxy=${http_proxy:-${HTTP_PROXY:-}}
export https_proxy=${https_proxy:-${HTTPS_PROXY:-}}
export no_proxy=${no_proxy:-localhost,127.0.0.1}
case "${1:-all}" in
 environment) docker build -t mohism-build:bookworm docker;exit;;
 host) CMD='cmake -S . -B out/host -G Ninja && cmake --build out/host -j8 && ctest --test-dir out/host --output-on-failure && python3 tests/mavlink_test.py && python3 tests/transport_test.py';;
 kernel) CMD='sh scripts/build-kernel.sh';;
 rootfs) CMD='sh scripts/prepare.sh && make -C vendor/buildroot-2023.02.6 O=/work/out/buildroot BR2_EXTERNAL=/work/buildroot/external mohism_rv1106_defconfig && make -C vendor/buildroot-2023.02.6 O=/work/out/buildroot BR2_EXTERNAL=/work/buildroot/external -j8';;
 refresh-rootfs) CMD='sh scripts/prepare.sh && make -C vendor/buildroot-2023.02.6 O=/work/out/buildroot BR2_EXTERNAL=/work/buildroot/external mohism-vins-dirclean && make -C vendor/buildroot-2023.02.6 O=/work/out/buildroot BR2_EXTERNAL=/work/buildroot/external -j8';;
 uboot) CMD='sh scripts/build-uboot.sh';;
 all) "$0" host;"$0" kernel;"$0" uboot;"$0" rootfs;"$0" package;exit;;
 package) CMD='sh scripts/package.sh';;
 *) echo 'usage: scripts/build.sh environment|host|kernel|uboot|rootfs|refresh-rootfs|package|all';exit 2;;
esac
docker run --rm --network host -e http_proxy -e https_proxy -e no_proxy -u "$(id -u):$(id -g)" -v "$PWD:/work" -w /work mohism-build:bookworm sh -c "$CMD"
