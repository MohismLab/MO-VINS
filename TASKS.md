# 执行清单

- [completed] 从 EDA 全工程网表确认硬件、引脚、电源和启动介质；记录 DRC 未通过、IO 电压、相机地址等待验证项。
- [completed] 固定并拉取 RV1106 SDK（kernel / U-Boot / Buildroot）。
- [completed] 编译设备树、内核、U-Boot、DDR/SPL、Buildroot 根文件系统和 SD 镜像。
- [completed] 实现 USB CDC ACM + UVC、MAVLink 8 参数服务；伪串口/数据打包测试通过。
- [completed] LSM6DSO 下沉内核，提供 /dev/imu0、poll、IRQ 时间戳和异常标志。
- [completed] 实现双目深度和松耦合视觉惯性原型；标定预览、标定门禁与失效处理。
- [completed] 主机测试、交叉编译、ENV/FIT/SD 布局、镜像文件一致性与 ext4 离线检查。
- [completed] 移植固定版本 VINS-Fusion 特征跟踪和中点预积分；补充数值/时间/几何检查、故障恢复、回放和回归测试。
- [not-started] VINS-Fusion 紧耦合滑窗、边缘化/回环；真实数据集及真值精度评估。
- [completed] 第二 USB ACM 调试串口、Linux getty/日志、U-Boot CDC 控制台；编译及离线集成检查通过。
- [completed] 多档高速请求、SDK 120fps 模式选择、稀疏/稠密预算分离、MAVLink 新估计直发与运行统计；合成验证通过，实板速率未测。
- [completed] PREEMPT_RT ARM 移植、完整补丁与源码哈希校验、IMU 硬中断时间戳保留、延迟测试工具；实板延迟与驱动运行待验证。
- [in-progress] 共同硬件同步接口核查、改线方案和产品验收基线；EDA Bridge 当前无客户端，改版与新驱动未完成。
- [not-started] 真板启动、USB 枚举、同步/标定和 VIO 精度验收（需要实物）。

目前交付为可构建的软件原型，最后一项尚未完成，因此不标记整体需求已验收。
