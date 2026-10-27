# LSM6DSO `/dev/imu0`

实现：`kernel/mohism_imu.c`。SPI0 CS0，SPI mode 3，1MHz；GPIO1_B0 接 INT1。驱动校验 WHO_AM_I=0x6c，复位并打开 BDU/自动地址递增，配置加速度与陀螺仪416Hz、±2g/±250dps，仅以 gyro DRDY 触发中断。

硬中断记录 `ktime_get_ns()`（CLOCK_MONOTONIC），线程化 IRQ 通过一次连续读取获得 gyro/accel 输出，送入512记录队列。设备支持阻塞读取、O_NONBLOCK 和 poll；只允许一个消费者，其他 open 返回 EBUSY。驱动移除后唤醒读取者，已打开句柄通过 kref 保持内存生命周期。

ABI 定义在 `include/mohism_imu.h`，v1 每记录固定32字节，当前ARM/x86使用little endian：

|偏移|类型|内容|
|---|---|---|
|0|uint64|IRQ 单调时钟 ns|
|8|uint32|驱动样本序号|
|12|uint32|flags|
|16|int16[3]|gyro 原始 XYZ；每LSB 0.00875°/s|
|22|int16[3]|accel 原始 XYZ；每LSB 0.000061g|
|28|uint32|保留，为0|

read长度必须是32的正整数倍，可以返回多记录；无数据时阻塞或EAGAIN。flags：1=队列溢出，2=SPI读取错误，4=中断时间间隔异常/样本未同时就绪。丢弃旧记录时保留最新样本；序号间断和flags均使本轮VIO终止，避免静默跨越丢样积分。用户态将原始量转换为m/s²与rad/s，再应用标定的Rbi。

此版没有启用芯片FIFO/硬件时间戳，也未实现休眠恢复、动态采样率或在线量程切换。硬中断时间不等于采样时间；4.1ms间隔检查只能发现部分异常，不是零丢样证明。负载下必须测量IRQ延迟、采样间隔分布、丢样率并标定时间偏移；更高精度可后续加入FIFO硬件时间戳重建。

板端检查示例（会独占并消费样本，先停止VIO）：

```sh
/etc/init.d/S50mohism-vins stop
ls -l /dev/imu0
dmesg | tail
dd if=/dev/imu0 bs=32 count=416 of=/tmp/imu.bin
/etc/init.d/S50mohism-vins start
```
