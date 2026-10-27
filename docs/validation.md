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
