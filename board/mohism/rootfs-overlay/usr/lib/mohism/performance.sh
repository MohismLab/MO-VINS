#!/bin/sh
# Trusted root-owned configuration. Shared by the gadget and application startup.
. "${MOHISM_PERFORMANCE_CONFIG:-/etc/mohism/performance.conf}"
case "$CAPTURE_FPS" in 15|30|60|120) ;; *) echo 'Invalid CAPTURE_FPS' >&2;return 1;; esac
case "$USB_FPS" in 15|30|60) ;; *) echo 'Invalid USB_FPS' >&2;return 1;; esac
case "$DEPTH_FPS" in 15|30|60|120) ;; *) echo 'Invalid DEPTH_FPS' >&2;return 1;; esac
[ "$USB_FPS" -le "$CAPTURE_FPS" ] && [ "$DEPTH_FPS" -le "$CAPTURE_FPS" ] || {
 echo 'USB_FPS/DEPTH_FPS cannot exceed CAPTURE_FPS' >&2;return 1;
}
USB_INTERVAL=$((10000000 / USB_FPS))
USB_BITRATE=$((384000 * 8 * USB_FPS))
FSIN_PERIOD=$((1000000000 / CAPTURE_FPS))
