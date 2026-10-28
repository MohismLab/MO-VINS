# 开发清单

## 已完成

- [x] 从 EDA 全工程网表提取引脚、电源和启动存储映射。
- [x] 固定 SDK、Buildroot、MAVLink 和算法上游版本。
- [x] 构建设备树、内核、U-Boot、DDR/SPL、根文件系统和 SD 镜像。
- [x] 双目 YUYV 与深度 Z16 两个 UVC 视频功能，MAVLink 与调试两个 CDC ACM 串口。
- [x] IMU 内核驱动 `/dev/imu0`：poll、IRQ 时间戳、样本序号和异常标志。
- [x] MAVLink 参数读写、持久化、状态与里程计输出。
- [x] 紧耦合双目/IMU 滑窗、逆深度优化、Schur 边缘化与运动初始化。
- [x] ORB 回环、四自由度位姿图、地图持久化及重定位。
- [x] 可选在线外参与时间偏移估计。
- [x] EuRoC MH_01_easy 全序列回放、分段真值评估与数值回归。
- [x] U-Boot CDC 控制台、Linux USB 日志与登录终端。
- [x] 多档采集与输出配置、稀疏跟踪与稠密深度独立调度、运行统计。
- [x] PREEMPT_RT ARM 移植、补丁哈希校验、模块 ABI 检查及延迟测试工具。
- [x] I²C1 后备 RTC 驱动、设备树、启动校时与构建检查。
- [x] 主机回归、ASan/UBSan、ARM QEMU、镜像布局与文件一致性检查。

## 后续开发

- [ ] 共同同步电路与驱动：DEN、门控、事件反馈及匹配。
- [ ] IMU FIFO、芯片时间戳与采样时间重建。
- [ ] 多数据集、弱纹理、动态场景、振动与长期运行评估。
- [ ] 服务自动重启、看门狗与升级回滚。
- [ ] SDIO Wi-Fi 接入与 H.264 无线预览。

测试项目见 [固件交付与测试](docs/product-delivery.md)，构建记录见 [验证记录](docs/validation.md)。
