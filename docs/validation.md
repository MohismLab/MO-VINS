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
RV1106实测帧率/CPU/RSS或长期运行证据；松耦合后端、没有滑窗/回环等限制仍存在。

Sanitizer最终通过记录来自 `sh scripts/build.sh sanitize` 对应的非PIE ASan/UBSan配置。
一次PIE测试启动挂起已终止，原日志独立保留，未将其记为通过；根因尚未确认。
