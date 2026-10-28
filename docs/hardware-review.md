# RV1106G3-VINS 硬件与固件核对

依据：2026-10-27 通过嘉立创 Bridge 的 `bridge_clients` 确认目标工程，通过 `schematic_review` 读取全工程网表。原始证据在 `hardware/netlist.json`；BOM 在 `hardware/BOM.md`。全工程 `drcCheckPassed=false`，接口未返回逐条违规。RTC 新增连线见 [RTC 配置](hardware/rtc.md)。

## 子系统

|子系统|全工程证据|固件处理|
|---|---|---|
|计算|U1 RV1106G3，X1 24MHz|ARMv7 Cortex-A7，SDK DDR/SPL + U-Boot + Linux 5.10.160|
|双目|U4/U12 OV9281，每颗两条 MIPI 数据线与独立时钟对|DPHY1/2 分拆输入、CSI0/1、CIF 原始灰度采集，绕开 Bayer ISP|
|惯性|U3 LSM6DSOWTR，SPI 与 INT1|专用 SPI 内核驱动，`/dev/imu0`|
|USB|USB1 Type-C，DP/DM 接 U1 27/26，25 接 VBUS_SENSE|USB2 peripheral，UVC + CDC ACM|
|启动存储|U6 TF 卡；U1 eMMC/FSPI 引脚未接存储器|SD 卡启动；不使用参考板 SPI NAND|

## 器件

|器件组|判断|限制|
|---|---|---|
|U1、X1、R130 240Ω|主控和 24MHz 时钟与 SDK 路径匹配；ZQ 接地电阻可见|实际 DDR 容量、DDR 训练与时钟稳定性需启动日志验证|
|U4/U12、U13/U14 镜头座|双目全局快门路线适合跟踪；SID 分别通过 R4 拉高、R28 拉低|通过双目标定获取基线、焦距、内参和外参|
|U3 LSM6DSO|SPI 和 INT1 接线支持独立采样驱动|IRQ 时间不是芯片内部采样时间；外参、时延与噪声需标定|
|U2 EA3059QDR、L1–L4、U7–U9|四路开关电源加相机 LDO|反馈参数、峰值电流和热裕量须按厂商规格实测|
|其余无源件、连接器、安装孔|完整列表已逐项导出到 BOM|仅网表不能验证 PCB 封装、走线和装配|

## 电源

|电源|连接|检查项目|
|---|---|---|
|5V|USB VBUS、U2 输入、U7 输入与扩展口供电|全板启动峰值及 USB 供电预算；固件声明 500mA 不会自动使主机授权更大电流|
|3.3V|TF、SoC 若干 IO 域、U3 VDD/VDDIO|用示波器确认上电时序和纹波|
|1.8V 系统|PLL/模拟等电源与相机 SCCB 上拉|核对 GPIO3 IO 域与相机 1.8V IO 的输入阈值和输出耐压|
|DDR 1.35V、核心 0.9V|U1 DDR_VDDQ 与 DVDD/CPU_DVDD|对应 R10–R17 反馈网络，核实输出精度与启动时序|
|相机 2.8/1.8/1.2V|U7 ME6211C28；U8/U9 从 2.8V 生成 1.8/1.2V|两颗相机共享供电，电源关断影响两路|
|VBUS 检测|R132 20kΩ 串联至 U1 USB_VBUSDET|按 RV1106 输入规格检查电压与限流|

## 引脚映射

|网名|实际 U1 引脚与功能|设备树|
|---|---|---|
|I2C2_SCL/SDA|6 / 8，GPIO3_D1/D2，I2C3 M2|`&i2c3`, `i2c3m2_xfer`|
|SPI2_SCK/MOSI/MISO/CS|94 / 93 / 92 / 95，SPI0 M0|`&spi0`, CS0|
|IMU_INT|77，GPIO1_B0，U3 INT1|上升沿，线程化 IRQ|
|SOC_UART0_TX/RX|79 / 80，UART2 M1|调试 UART2，ttyS2|
|SOC_UART1_TX/RX|75 / 76，UART1 M0|独立 UART1|
|SHUTDOWN1/2|3 / 5，GPIO3_C5/C7|OV9281 驱动 `pwdn-gpios`；此 SDK 的逻辑 1 表示上电，需示波器确认实际高电平|
|XVCLK1/2|2 / 4，MIPI_CLK0/1_OUT|24MHz MCLK_REF_MIPI0/1|
|FSIN|9，GPIO3_D3 / PWM1 M2，两颗相机共同 FSIN|按 performance.conf 请求硬件 PWM（当前120Hz）、10µs 脉冲；传感器补丁启用外触发，时序需实测|
|左 MIPI|127/128 D0、123/124 D1、125/126 CLK|DPHY1，逻辑 data-lanes=<1 2>|
|右 MIPI|121/122 D2、117/118 D3、119/120 CLK1|DPHY2，分拆后逻辑 data-lanes=<1 2>|
|SID|U4 E1 上拉 1.8V，U12 E1 下拉|暂按唯一 7-bit 地址左 0x10、右 0x60；**需读芯片 ID 验证**，不能用广播地址区分传感器|

## 保护与故障处理

|核查点|网表可确认|结论|
|---|---|---|
|USB CC|R108/R109，各 5.1kΩ 下拉|存在设备端 CC 下拉|
|ESD、差分阻抗、地回流|结合 PCB 布局检查 ESD 器件、差分线与回流路径|检查器件位置、阻抗与地连续性|
|丢样检测|内核队列溢出、SPI 错误、IRQ 间隔异常都有标志；用户态检查序号|驱动使用输出寄存器；负载测试记录中断延迟、序号间断和异常标志|
|时钟与同步|共用 FSIN，IMU 与 CIF 采用单调时钟|CIF 时间戳含义、曝光中心偏移和 IMU 中断偏移均需标定|
|标定与故障|默认 `calibrated: 0`，不输出有效位姿；异常则停止本次估计|完成标定后启用有效位姿输出|

## 固件集成

|层面|状态|
|---|---|
|板级接线映射|已从当前 EDA 工程提取，并写入可编译设备树|
|启动/驱动构建|内核、设备树、专用 IMU 模块、U-Boot/DDR/SPL 可生成|
|软件功能验证|合成双目尺度、视觉平移、IMU 与 MAVLink 伪串口测试；USB 深度字节往返测试|

## 同步连线

导出网表中的FSIN只有三端：SoC 9脚 GPIO3_D3/PWM1_M2，以及两颗OV9281的B5脚
FSIN/VSYNC。板级DTS启用PWM1 M2，两颗相机均带 `mohism,external-fsin` 属性；
启动脚本按 performance.conf 设置频率（当前请求120Hz）、10µs高电平脉冲，驱动补丁在STREAMON前写外部FSIN配置。
左右曝光时差通过 FSIN 与曝光波形测量。

U3 LSM6DSO的4脚INT1接IMU_INT；9脚INT2在当前网表中未连接，也没有接入FSIN。
驱动由陀螺DRDY触发，硬IRQ记录CLOCK_MONOTONIC时间，线程读取加速度/角速度。
它不是相机与IMU共用硬件触发；IRQ时间也不是传感器内部采样时刻。
当前算法依赖标定的imu_offset、相邻采样插值及匹配时间戳。
软件图像时间差门限为 2ms。

共同事件同步的连线和驱动设计见 [同步方案](hardware-sync.md)，测试项目见 [交付检查](product-delivery.md)。
