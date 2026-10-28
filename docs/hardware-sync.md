# 相机与 IMU 共同硬件同步改版

本文描述共同事件同步的改线方案。当前固件使用共用 FSIN 触发双目、INT1 读取 IMU；下文 DEN、门控和反馈路径属于新增设计。引脚依据为 `hardware/netlist.json`。

## 当前连线与必须处理的问题

现有 U3 INT1（4 脚）接 U1 GPIO1_B0（77 脚），中断已接；INT2（9 脚）没有网络。左右相机 B5 FSIN 共接 U1 GPIO3_D3（9 脚）。U1 GPIO3_VCC（43 脚）和 U3 VDDIO（5 脚）接 3.3V，相机 IO 电源为 1.8V。FSIN 跨电源域连接需加入电平转换。

ST 数据手册 DS12140 Rev 2 的 Table 2 / Table 24 表明 INT2 可用作 DEN，且复位默认输出低；Table 59 定义 DEN 触发模式，Table 67 描述把 DEN 信息写入传感器数据最低位。官方寄存器驱动也提供 DEN 配置接口。DEN 是数据标记/选通功能，不能由此推导 IMU ADC 与相机曝光严格同相或滤波延迟为零。

## 建议连线

|网络|端点与作用|
|---|---|
|SYNC_PWM_3V3|U1.9 GPIO3_D3/PWM1_M2，硬件脉冲源|
|SYNC_REF_3V3|门控后的共同事件，接 U3.9 INT2/DEN 与 U1.78 GPIO1_B1 反馈输入|
|CAM_FSIN_1V8|共同事件经过固定方向电平转换，接 U4.B5、U12.B5|
|SYNC_ENABLE|U1.73 GPIO1_A1 控制门控；外部下拉使上电默认关闭|
|IMU_INT|保持 U3.4 → U1.77，独立报告 IMU 数据就绪|

U1.78 和 U1.73 在当前网表为空闲，仍需 PCB/复用/启动上下文核查。门控输出在 IMU 配置完成前必须保持禁用，避免对复位时输出低的 INT2 强推高电平。电平转换器应按上电顺序、掉电容忍、输出使能和脉冲时序选型，使用适合推挽脉冲的固定方向缓冲器。

共同事件从 PWM 经门控后同时到达 IMU DEN、SoC 反馈和相机电平转换路径。相机间共享同一 FSIN 支路；测试点必须覆盖门控前后和相机端。既有 SCCB、PWDN 等跨 IO 电源域信号也须一并核对。

## 驱动流程

1. 驱动上电先禁止门控，复位并配置 IMU DEN 输入、极性、标记轴和采样/FIFO；确认没有在 INT2 路由输出中断，再开放共同脉冲。关机、解绑、超时和恢复先关门控。
2. 从 FIFO 获取加速度、角速度及芯片时间戳；识别并剥离占用数据最低位的 DEN 标志，另行输出事件序号。驱动异常、FIFO 溢出与事件丢失必须上报并停止当前估计。
3. 反馈通路关联触发计数、左右帧序号、DEN 标志与同一时间基准。GPIO IRQ 记录中断到达时间。RV1106 PWM 支持 capture，但 SDK PWM 驱动未提供对应实现，硬件捕获需扩展驱动。
4. 标定 FSIN 到左右实际曝光中心、DEN 标记对应采样及 IMU 滤波群延迟。ADC 自由运行与共同事件标记的差别必须反映在误差预算；严格锁相需要支持相应采样控制的硬件。
5. 算法要求可验证的事件匹配；断线、错序、复位、温漂或长时间漂移越界时撤销有效 VIO 输出。软件时延参数用于补偿固定偏移。

上述新路径尚未在驱动中启用，当前固件仍采用 INT1 IRQ 时间戳与软件时延标定。连线检查脚本 `scripts/check-hardware-sync.py` 会对现有网表失败，直到新连线导出并核对；检查范围为网络端点。

## 参考证据

- ST DS12140 Rev 2（LSM6DSO）：官方原始地址 `https://www.st.com/resource/en/datasheet/lsm6dso.pdf`；本次从 SparkFun Documentation 中的同名 ST 原始文档副本读取，副本地址记录在 `hardware/sync-references.json`。版本以该记录为准。
- ST 官方 `lsm6dso-pid`，固定 commit `b45daae59111e8856544e7d2b832ad684e3d799e`，用于核对寄存器语义。
- RV1106 原有 datasheet Timer/PWM 部分；本工程锁定 SDK 的 `drivers/pwm/pwm-rockchip.c` 和 `arch/arm/boot/dts/rv1106-pinctrl.dtsi`。
