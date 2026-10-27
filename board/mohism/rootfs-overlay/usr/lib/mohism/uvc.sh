#!/bin/sh
# Descriptor creation shared by gadget startup and offline fixture tests.
  create_uvc() {
   instance=$1 width=$2 bytes=$3 packet=$4 interval=$5 bitrate=$6
   F=$G/functions/$instance
   mkdir -p "$F"
   echo "$instance" > "$F/function_name"
   echo 1 > "$F/streaming_interval"
   echo "$packet" > "$F/streaming_maxpacket"
   P=$F/streaming/uncompressed/u
   mkdir -p "$P/${width}x200"
   echo 16 > "$P/bBitsPerPixel"
   if [ "$instance" = uvc.depth ]; then
    # UVC Z16 GUID, little-endian uint16 depth in millimetres.
    printf 'Z16 \000\000\020\000\200\000\000\252\000\070\233\161' > "$P/guidFormat"
   fi
   R=$P/${width}x200
   echo "$width" > "$R/wWidth"
   echo 200 > "$R/wHeight"
   echo "$bitrate" > "$R/dwMinBitRate"
   echo "$bitrate" > "$R/dwMaxBitRate"
   echo "$bytes" > "$R/dwMaxVideoFrameBufferSize"
   echo "$interval" > "$R/dwDefaultFrameInterval"
   echo "$interval" > "$R/dwFrameInterval"
   mkdir -p "$F/streaming/header/h"
   ln -s ../../uncompressed/u "$F/streaming/header/h/u"
   ln -s ../../header/h "$F/streaming/class/fs/h"
   ln -s ../../header/h "$F/streaming/class/hs/h"
   mkdir -p "$F/control/header/h"
   ln -s ../../header/h "$F/control/class/fs/h"
   ln -s ../../header/h "$F/control/class/ss/h"
  }
