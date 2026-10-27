# USB 调试串口

Linux 固件提供一个 UVC 视频功能和两个独立 CDC ACM 串口：

|用途|板端节点|USB 接口号|主机常见节点|
|---|---|---|---|
|MAVLink 参数与里程计|`/dev/ttyGS0`|控制 2、数据 3|`/dev/ttyACM0`|
|内核日志、登录与 Shell|`/dev/ttyGS1`|控制 4、数据 5|`/dev/ttyACM1`|

UVC 控制/视频接口仍为 0/1。主机 ttyACM 编号会受其他设备和重新枚举影响；请通过 `udevadm info -q property -n /dev/ttyACM<N>` 的设备标识和 `ID_USB_INTERFACE_NUM` 区分角色，不能只依赖编号。Linux 开发 VID/PID 为 `1d6b:0104`。

调试端口使用终端程序，例如 `picocom -b 115200 /dev/ttyACM1`，打开后按回车出现登录提示。CDC 的波特率参数不决定 USB 传输速度。Buildroot 当前是开发镜像，root 没有密码；发布前需配置密码。BusyBox getty 提供登录与 Shell，内核仅在 ttyGS1 注册 USB console。ACM 实例默认自动注册 console 的 SDK 行为已修正，避免日志混入 MAVLink 字节流。UART2 的 ttyS2 console 仍保留。

## U-Boot 阶段

SDK 原有 USB loader 是下载协议，不是串口。本工程加入独立 CDC ACM 控制台 `usbdbg`，与 UART `serial` 同时作为输入输出。U-Boot 使用 `1d6b:0105`、产品名 `MOHISM U-Boot Debug`，此阶段仅一个调试 ACM。启动倒计时为 5 秒，可从 USB 或 UART 打断并执行 U-Boot 命令；主机未打开端口时启动不等待。

USB 控制台在 U-Boot 重定位后的 stdio 初始化时建立，不能输出 BootROM、DDR 初始化、SPL 或此前的日志。这些阶段请使用 UART2。USB 输出保留最近约 32KiB，未连接时较早内容会被覆盖。控制台通过轮询处理 USB，长时间不轮询的命令可能延迟 USB 通信。

跳转 Linux 前注销 U-Boot gadget 并释放控制器，Linux 启动后重新枚举为 UVC + 两个 ACM。原终端连接会断开，需要按新的设备节点重连；不能保证跨启动阶段保持同一 ttyACM 编号。Linux USB gadget 在 init 阶段建立，也不能替代早期 UART console。

## 验证范围

已提供 ARM U-Boot、内核与 rootfs 构建，以及 `scripts/verify-usb-debug.py` 对实际构建配置、ELF 符号、DTB、登录配置和镜像的检查。它们不能证明物理设备正确枚举或串口收发正常。

上板须验证：U-Boot ACM 枚举、5 秒内打断启动、命令双向收发、无主机也能启动、Linux 切换重枚举、UVC 与两个 ACM 并发、MAVLink 无日志污染、Shell 登录、内核日志、反复插拔与 gadget 重启。当前未连接目标硬件，以上实测尚未完成。
