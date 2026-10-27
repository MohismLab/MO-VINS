/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef MOHISM_IMU_UAPI_H
#define MOHISM_IMU_UAPI_H
#include <linux/types.h>
/* ABI v1: 32-byte native little endian records. CLOCK_MONOTONIC ns at IRQ.
 * +/-2g accel: 61 micro-g/count. +/-250dps gyro: 8750 micro-deg/s/count. */
#define MOHISM_IMU_ABI_VERSION 1
#define MOHISM_IMU_FLAG_OVERFLOW 1U
#define MOHISM_IMU_FLAG_SPI_ERROR 2U
#define MOHISM_IMU_FLAG_TIMING_GAP 4U
struct mohism_imu_sample {
 __u64 timestamp_ns;
 __u32 sequence, flags;
 __s16 gyro[3], accel[3];
 __u32 reserved;
};
#endif
