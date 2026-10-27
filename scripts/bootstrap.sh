#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
. ./scripts/env.sh
SDK_COMMIT=824b817f889c2cbff1d48fcdb18ab494a68f69d1
MAVLINK_COMMIT=6b4243f0e54b42ddb1e65df7a3a5e8c99d15ec1f
mkdir -p vendor .cache
if [ ! -f vendor/luckfox-pico/sysdrv/source/kernel/Makefile ]; then
 curl -fL --retry 2 "https://codeload.github.com/LuckfoxTECH/luckfox-pico/tar.gz/$SDK_COMMIT" -o .cache/sdk.tar.gz
 echo '5bf45ea0bc1cd524bd52b5004e87556a49bb8f579223ee906bd0cd3e5ccae735  .cache/sdk.tar.gz' | sha256sum -c -
 mkdir -p vendor/luckfox-pico
 tar -xzf .cache/sdk.tar.gz --strip-components=1 -C vendor/luckfox-pico
 echo "$SDK_COMMIT" > vendor/luckfox-pico/.source-commit
fi
if [ ! -d vendor/mavlink ]; then
 git init vendor/mavlink
 git -C vendor/mavlink remote add origin https://github.com/mavlink/c_library_v2.git
 git -C vendor/mavlink fetch --depth 1 origin "$MAVLINK_COMMIT"
 git -C vendor/mavlink checkout --detach FETCH_HEAD
fi
test "$(git -C vendor/mavlink rev-parse HEAD)" = "$MAVLINK_COMMIT"
if [ -d vendor/luckfox-pico/.git ]; then
 test "$(git -C vendor/luckfox-pico rev-parse HEAD)" = "$SDK_COMMIT"
else
 test "$(cat vendor/luckfox-pico/.source-commit)" = "$SDK_COMMIT"
fi
if [ ! -f vendor/buildroot-2023.02.6/Makefile ]; then
 tar -xzf vendor/luckfox-pico/sysdrv/tools/board/buildroot/buildroot-2023.02.6.tar.gz -C vendor
fi
echo 'Pinned SDK, MAVLink and Buildroot available.'
