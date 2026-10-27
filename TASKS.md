# 执行清单

- [completed] 从 EDA 全工程网表确认硬件、引脚、电源和启动介质；记录 DRC 未通过、IO 电压、相机地址等待验证项。
- [completed] 固定并拉取 RV1106 SDK（kernel / U-Boot / Buildroot）。
- [completed] 编译设备树、内核、U-Boot、DDR/SPL、Buildroot 根文件系统和 SD 镜像。
- [completed] 实现 USB CDC ACM + UVC、MAVLink 8 参数服务；伪串口/数据打包测试通过。
- [completed] LSM6DSO 下沉内核，提供 /dev/imu0、poll、IRQ 时间戳和异常标志。
- [completed] 实现双目深度和松耦合视觉惯性原型；标定预览、标定门禁与失效处理。
- [completed] 主机测试、交叉编译、ENV/FIT/SD 布局、镜像文件一致性与 ext4 离线检查。
- [not-started] 真板启动、USB 枚举、同步/标定和 VIO 精度验收（需要实物）。

目前交付为可构建的软件原型，最后一项尚未完成，因此不标记整体需求已验收。
