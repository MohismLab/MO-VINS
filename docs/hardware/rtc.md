# 后备实时时钟

2026-10-28 通过 EDA Bridge `schematic_review` 读取全工程网表，核对本次新增的 RTC。未修改原理图；网表返回 DRC 未通过，不据此认定整板已通过电气审查。

| 检查项 | 网表结果及处理 |
| --- | --- |
| 电路功能 | U25 PCF8563TS 提供后备日历时钟；图像和 IMU 同步仍使用 CLOCK_MONOTONIC。 |
| 器件与选型 | X2 为 32.768 kHz 晶振；C5/C6 各 15 pF。晶振有效负载包含芯片内部及 PCB 寄生电容，量产前需检查起振与走时误差。 |
| 电源方案 | U17 BAT54C 将主电源和 U18 后备电池汇合到 VDD_RTC，再经 R5 4.7 kΩ 到 U25.8；C4 100 nF 对地。网表存在完整连接。此处未验证后备续航和电池充电条件。 |
| 信号与连线 | U1.64 GPIO0_A5 → I2C1_SCL → U25.6；U1.65 GPIO0_A6 → I2C1_SDA → U25.5。R1/R2 各 1.5 kΩ 上拉到 VDD3V3_SYS。OSCI/OSCO 分别连 X2 两端，VSS 接地。 |
| 保护与可靠性 | INT 与 CLKOUT 未使用。软件未声明中断、闹钟唤醒或时钟消费者；低电压标志导致读时返回错误，必须重新校时。 |
| 可用性评估 | 本次 RTC 软件配置与网表对应。其余页面不属于本次功能验证范围；整板 DRC、断电保持、总线波形和走时仍需实板验收。 |

## 内核配置

设备树 `board/mohism/rv1106g3-mohism-vins.dts` 启用 I2C1 M0，100 kHz，添加 `nxp,pcf8563`、7 位地址 `0x51`，并以 `rtc0` 别名指定外部时钟。片内 RTC 保持禁用，避免编号竞争。

`board/mohism/kernel.config` 将 RTC core、字符设备、PCF8563 与 RK3X I²C 驱动编入内核，启用 `RTC_HCTOSYS_DEVICE="rtc0"`。有效 RTC 在内核启动阶段恢复系统 UTC 时间，无需额外启动脚本。未启用周期性系统时间回写，避免把未经校准的时间覆盖到后备时钟。

首次上电或后备电池耗尽时，驱动可能报告 `low voltage detected, date/time is not reliable`。这表示时间无效，不应作为有效日期使用。先取得可信 UTC 时间，再写入：

```sh
# 将下列示例替换为实际 UTC 时间
 date -u -s '2026-10-28 12:00:00'
 hwclock -w -u -f /dev/rtc0
 hwclock -r -u -f /dev/rtc0
```

板端验收：

```sh
cat /sys/class/rtc/rtc0/name
cat /sys/class/rtc/rtc0/hctosys
dmesg | grep -E 'pcf8563|rtc|hctosys'
hwclock -r -u -f /dev/rtc0
```

预期设备名称包含 `pcf8563`，有效时间启动后 `hctosys` 为 `1`。主电源断开、保留后备电池，等待后重新启动，检查 UTC 时间连续递增。INT 未接，不验收闹钟唤醒。尚无生产实板，以上是验收步骤，不是已完成的硬件测试。

依据：SDK 内的 `Documentation/devicetree/bindings/rtc/pcf8563.txt`、`drivers/rtc/rtc-pcf8563.c`、`arch/arm/boot/dts/rv1106-pinctrl.dtsi`；NXP《PCF8563 Real-time clock/calendar》中的 I²C 读写地址 A3h/A2h 对应 Linux 的 7 位地址 0x51。
