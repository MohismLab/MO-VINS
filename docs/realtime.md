# PREEMPT_RT

默认内核构建已改为 PREEMPT_RT。移植基于固定版本的厂商 Linux 5.10.160，将官方 5.10.158-rt77 的实时补丁适配到该源码，内核版本标记为 `5.10.160-rt77-mohism`。这是本工程的移植版本，不是上游原版发行，也没有借此升级 Linux 基线。

## 构建

```sh
sh scripts/build.sh kernel
sh scripts/build.sh rootfs
sh scripts/build.sh package
```

`kernel/rt/preempt-rt-arm.patch` 包含实际的实时锁、调度、软中断、高精度定时器、ARM 入口和 printk 改动。`kernel/rt/manifest.json` 记录上游来源、补丁校验值，以及每个文件修改前后的哈希。构建时先核验 SDK，再生成独立的 `out/kernel-rt-src-<hash>/`；原始 SDK 源码保持不变。重复构建会验证生成源码的哈希，文件被额外修改时停止构建。

板级功能配置仍在 `board/mohism/kernel.config`，实时配置在 `board/mohism/realtime.config`。主要选项为：

```text
CONFIG_PREEMPT_RT=y
CONFIG_HIGH_RES_TIMERS=y
CONFIG_HZ=1000
CONFIG_DEBUG_ATOMIC_SLEEP=y
CONFIG_DEBUG_PREEMPT=y
```

当前配置保留运行期原子上下文和抢占检查，方便首板暴露驱动问题。普通自旋锁在 RT 下变为可睡眠的优先级继承锁；底层中断和调度临界区使用原始自旋锁。中断按 RT 机制线程化，日志使用补丁提供的控制台打印线程。编译检查同时核对链接后的 RT 函数符号，而不是只检查 `.config`。

已停用板上未使用的 FIQ 调试驱动及厂商独立的软中断优化选项，调试仍使用普通 UART 和 USB CDC。端口适配记录见 [补丁说明](../kernel/rt/README.md)。

内核结构和模块 ABI 已变化。IMU 及 SDK 中启用的模块均须针对 RT 内核重新编译，不能沿用普通内核的 `.ko`。后续 Wi-Fi 和视频编码模块也必须验证 RT 兼容性，尤其不能直接将厂商预编译模块当作已适配。

## IMU 中断

IMU 驱动继续使用显式的两段式中断：

1. 硬中断入口读取单调时钟，记录中断到达时间。
2. 中断线程读取 SPI、检查状态并写入样本队列。

注册中断时使用 `IRQF_NO_THREAD`，避免 RT 把第一步再次强制线程化；它不取消显式注册的 SPI 读取线程。硬中断入口不进行 SPI 通信，也不获取普通自旋锁。

该时间戳仍是 CPU 观察到的中断时刻，不等于传感器实际采样时刻。PREEMPT_RT 不能替代共同硬件同步，也不会自动完成相机与 IMU 的时钟标定。

## 板端检查

启动新固件后执行：

```sh
uname -r
cat /sys/kernel/realtime
mohism-rt-check
```

预期内核版本为 `5.10.160-rt77-mohism`，`/sys/kernel/realtime` 为 `1`。检查工具读取内核配置、RT 带宽设置，以及 IMU/SPI 中断计数和 IRQ 线程调度策略，不修改线程优先级。

根文件系统包含 rt-tests 的 `cyclictest`。先记录空载基线：

```sh
cyclictest --mlockall --priority=80 --interval=1000 --duration=10m --quiet
```

再在双目采集、VIO、深度、USB 视频和两个串口同时工作时运行同一命令，记录完整输出、固件版本、负载与温度。测试程序本身使用高优先级，会影响应用调度；频率和优先级也应写入测试记录。重启后执行 `dmesg`，检查 sleeping-in-atomic、调度、锁、USB、SPI 和 CIF 报错。

重点观察最大调度延迟、IMU 时间间隔和异常标志、图像丢帧、有效位姿更新率、USB 热插拔以及长时间温升。`cyclictest` 测量线程唤醒延迟，不测量曝光到位姿的端到端延迟，也不提供最坏执行时间证明。

没有把整个 VIO 进程直接提升到 `SCHED_FIFO`，没有关闭 RT 带宽限额，也没有固定 IMU/SPI 线程优先级。单核平台上的优先级设置需要结合实测，避免视觉计算饿死控制台、网络或必要的驱动线程。

## 普通内核回退

```sh
sh scripts/build.sh kernel-standard
```

回退构建输出到 `out/kernel-standard/`、`out/modules-standard/` 和 `out/artifacts/standard/`，不覆盖默认 RT 镜像。若要上板回退，需同时使用该内核对应的模块重建根文件系统和启动镜像，不能只替换 `zImage`。现有 `package` 命令检查并打包默认 RT 构建。

## 验证范围

本次移植可进行源码哈希、ARM 交叉编译、模块 ABI、根文件系统与镜像一致性检查。由于硬件尚未生产，尚未验证真实启动、负载调度延迟和外设运行，也没有产品实时性验收结论。实测数据应保留在对应硬件和固件版本的验收记录中。
