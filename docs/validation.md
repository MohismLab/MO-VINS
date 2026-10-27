# 2026-10-27 构建与验证记录

原理图来源为当前EDA工程RV1106G3-VINS，具体证据与限制见硬件报告。构建在Debian bookworm容器中完成，使用SDK自带ARM Rockchip GCC 8.3 / uClibc交叉工具链。

|检查|结果|证据|
|---|---|---|
|C++ 主机编译与核心算法CTest|通过|`out/validation/host-tests.log`|
|合成双目1.5m尺度、静止VO及3cm平移方向/尺度|通过|`tests/core_test.cpp`|
|未标定门禁、静止IMU积分、SO(3)、异常观测/时间间隔拒绝|通过|同上|
|MAVLink真实字节经PTY串口传输，CRC、8参数枚举、写入/只读/范围、重启持久化|通过|`tests/mavlink_test.py`、主机日志|
|左右灰度与16位深度随机字节往返|通过|`tests/transport_test.py`、主机日志|
|RV1106 Linux zImage、DTB、IMU模块|交叉编译通过|`out/validation/kernel-build.log`|
|U-Boot与DDR/SPL打包，备用扇区0x440|构建通过|`out/validation/uboot-build.log`|
|Buildroot ARM程序与ext4根文件系统|构建通过|`out/validation/buildroot-build.log`|
|ENV CRC、FIT嵌入内核/DTB SHA256、SD各偏移与源文件逐字节一致|通过|`scripts/verify-artifacts.py`、`out/validation/package.log`|
|镜像内程序/IMU模块与单独产物逐字节一致|通过|debugfs提取后cmp，`out/validation/artifacts-check.log`|
|ext4离线e2fsck五阶段检查、所有交付文件SHA256|通过|`out/validation/artifacts-check.log`|
|目标板启动、真实USB UVC/CDC枚举与热插拔|未测试|当前没有目标板连接|
|真实双相机曝光同步、相机地址/IO电压、DDR/电源|未测试|需实物、启动日志与仪器|
|真实标定、深度精度、VIO轨迹、帧率/CPU/内存/温度|未测试|无实测数据，默认禁止有效VIO输出|

已生成 `out/artifacts/mohism-sd.img`，尺寸319586304字节（约304.78MiB），可按bringup文档写入已确认的测试SD卡；未自动烧录设备。镜像格式和尺寸检查通过不代表已经证明硬件可启动。

内核`modules_install`提示容器缺少系统depmod；根文件系统post-build阶段已使用Buildroot的host-depmod生成模块依赖，镜像中的IMU模块已逐字节核对。此提示不被当作真板驱动运行证据。

# 2026-10-28 VINS-Fusion组件移植回归（0.2.0）

详细来源、实现边界和回放说明见 [算法移植与验证](vins-fusion-port.md)。

|检查|结果|证据|
|---|---|---|
|主机C++构建及5项CTest：core/robustness/replay/odometry/trajectory|通过|`out/validation/vins-fusion-host-tests.log`|
|原有MAVLink参数/持久化及16位图像传输测试|通过|同上|
|ASan及UBSan下5项CTest|通过|`out/validation/vins-fusion-sanitizers.log`|
|中点积分、偏置雅可比数值差分、重传播、协方差PSD、NaN与时间检查|通过|`tests/robustness_test.cpp`|
|完全遮挡、运动时拒绝初始化、静止恢复、禁用/启用和重置epoch|通过|同上及PTY ODOMETRY测试|
|合成EuRoC格式CSV/PNG回放65帧|45帧有效；初始阶段/遮挡/恢复等待均无效|`tests/replay_test.py`，CTest LastTest.log|
|30帧三深度层六自由度运动、亮度变化及局部遮挡|通过；本次最大位置误差0.003543m、姿态误差0.004506rad|`tests/trajectory_test.cpp`，CTest LastTest.log|
|新版ARM用户态及Buildroot/ext4根文件系统|交叉编译通过|`out/validation/vins-fusion-arm-build.log`|
|重新生成SD镜像、ENV/FIT/分区载荷一致性|通过|`out/validation/vins-fusion-package.log`|
|所有SHA256、e2fsck、镜像内程序与单独交付程序逐字节比较、许可证文件|通过|`out/validation/vins-fusion-artifacts.log`|
|源码归档独立解包后编译、5项CTest及MAVLink/传输测试|通过|`out/validation/vins-fusion-source-build.log`|

这些结果仅证明当前软件在指定合成输入与构建环境中通过测试。3.543mm等数值是短序列合成回归结果，
不代表OV9281/LSM6DSO实测精度。尚未跑真实EuRoC/TUM-VI或本板带真值数据，没有ATE/RPE、
RV1106实测帧率/CPU/RSS或长期运行证据；当时仍为松耦合后端；0.5.0 的更新结果见本页后续记录。

Sanitizer最终通过记录来自 `sh scripts/build.sh sanitize` 对应的非PIE ASan/UBSan配置。
一次PIE测试启动挂起已终止，原日志独立保留，未将其记为通过；根因尚未确认。

## 第二 USB 串口与 U-Boot CDC（2026-10-28）

U-Boot ARM 构建通过，实际配置启用 MOHISM_USB_CONSOLE、DWC3 gadget、console mux 和 5 秒倒计时。SDK 的旧板级 USB 初始化与 DM gadget 冲突、无 XHCI 时仍链接 host 驱动的问题已由幂等补丁处理。最终 uboot.img 仍为 262144 字节，满足既有 256KiB 分区。

内核与 rootfs 已重建；`scripts/verify-usb-debug.py` 核对实际内核/U-Boot 配置、ELF 中控制台与 DM 中断符号、编译 DTB 的 ttyGS1/ttyS2 console、唯一 getty、securetty、ACM 默认不注册 console 及镜像分区大小。host 的 5 项 CTest 与 MAVLink/传输 Python 测试通过。ENV/FIT/SD 镜像内容与哈希验证通过。日志位于忽略目录 `out/validation/usb-*.log`。

以上属于编译和离线验证，没有目标板 USB 枚举、U-Boot 命令交互或 Linux 并发传输实测，不标记实机验收完成。

## 高速开发配置与产品要求（2026-10-28）

0.3.0 请求采集 120fps、USB 60fps、稠密深度 30Hz，SDK 原有 120fps 模式通过板级属性选用；这些不是实测性能。新增 profile 测试使用 120Hz 合成时间戳确认稀疏 VO 连续、30Hz 稠密计算、跳过时深度清零，并检查统计文件的丢帧/序号重启/耗时分位数。PTY 测试确认新估计不再被原 50ms 间隔抑制，重复时间戳不制造消息。6 项 CTest、MAVLink 与传输测试通过；非 PIE ASan/UBSan 的 6 项测试通过。

内核、ARM 应用和根文件系统交叉编译通过；OpenCV 已完整重编译，实际 CMake cache 为 `ENABLE_NEON=ON`、`ENABLE_FAST_MATH=OFF`，编译参数末尾为 `-O3`，应用 Release 同样使用 `-O3 -fno-fast-math`。没有据此推算或宣称目标 CPU 的加速比例。`scripts/verify-performance.py` 核对各档周期/带宽、非法组合拒绝、两颗传感器的编译 DTB 标志及真实编译选项。

现有网表执行 `scripts/check-hardware-sync.py` 预期失败：INT2、反馈与门控仍未连接，FSIN 尚未分离为建议电压域。EDA Bridge 两次查询无客户端；同步改版未写入 EDA，不能声称已做共同硬件同步或新 DRC 通过。产品门槛与 RT 状态记录于 `docs/product-delivery.md`，未测项保持未完成。


## 双 UVC 视频功能（2026-10-28）

0.4.0 将旧的三块 YUYV 打包替换为两个 configfs UVC 功能：双目 640×200 YUYV，深度 320×200 Z16。保留 MAVLink 和调试两个 ACM。默认请求双目 60fps、深度 30fps；这些是描述符请求值。

7 项 CTest、MAVLink 伪串口测试、两个主机解码器测试及描述符文件系统夹具通过。测试覆盖左右字节布局、uint16 深度值、非连续图像内存、非默认接口号的控制请求路由、错误接口拒绝、两路独立统计，以及 Z16 GUID、几何尺寸、帧周期和功能链接顺序。非 PIE ASan/UBSan 的 7 项 CTest 通过。

PREEMPT_RT 内核、ARM 应用与根文件系统重建通过，离线镜像门禁通过。内核新增 Z16 gadget 格式支持，已同步到实际 RT 构建源码；检查同时核对安装后的两个启动脚本和描述符辅助脚本。日志为 `out/dual-uvc-{host,sanitize,kernel,rootfs,package}.log`。

尚未验证物理 USB 枚举、控制器端点数量、主机双路等时带宽分配、实际传输率、启停与热插拔。两路视频没有共享曝光元数据；单独保存的序号和主机到达时间不能用于跨流曝光配对。上述测试不替代实板验收。

## 0.5.0 紧耦合后端

本版启用双目/IMU 联合滑窗、逆深度优化与 Schur 边缘化，补充运动初始化、ORB 回环、四自由度图、持久地图及重定位；在线外参/时间估计默认关闭。

主机 11 项 CTest、MAVLink/传输/双 UVC 离线检查通过。ASan/UBSan 与 ARM QEMU 检查覆盖新增数学、初始化和地图路径。完整 Buildroot 固件重建后检查 ARM 动态依赖闭包及 Ceres 许可证。日志分别为 `out/tight-host.log`、`out/tight-sanitize.log`、`out/tight-arm-validation.log`、`out/tight-rootfs.log` 和 `out/tight-package.log`。

EuRoC MH_01_easy 全序列有效率 98.29%，25 次重置或修正计数变化；分段无尺度拟合的 ATE RMSE 为 0.05988m，1 秒平移/旋转 RPE RMSE 为 0.05476m/0.63353°。全程 ATE 不作有效结论。数据来源、逐段结果、复现方式和限制见 [算法验证](vins-fusion-port.md) 及 `qualification/euroc-mh01-*.json`。本次结果不代表实板精度或产品放行。
