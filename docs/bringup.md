# 构建、写卡与上板验收

板级引脚、电源和接口映射见 [硬件核对](hardware-review.md)。

## 重新构建

```sh
sh scripts/bootstrap.sh
sh scripts/build.sh environment
sh scripts/build.sh all
```

`out/host` 为 x86 主机测试，`out/kernel` 为独立内核构建目录，`out/buildroot` 为交叉构建根文件系统，`out/artifacts` 为镜像。SDK 在 `vendor/luckfox-pico`，Buildroot 在 `vendor/buildroot-2023.02.6`，MAVLink 在 `vendor/mavlink`。所有容器以用户UID运行，源工程不要求搬出中文路径。下载使用已配置的HTTP(S)代理（若存在）和host网络；没有代理时直接访问。

若已构建后修改 `src/` 或 `include/`，需清除本地包stamp再构建。可直接运行 `sh scripts/build.sh refresh-rootfs`，然后 `sh scripts/build.sh package`。板级内核改动运行 `sh scripts/build.sh kernel`，默认构建 PREEMPT_RT；普通内核比较使用 `kernel-standard`。实时检查与延迟测试见 [实时内核](realtime.md)。U-Boot改动运行 `sh scripts/build.sh uboot`。

## 镜像布局

由 `scripts/make-sd-image.py` 生成ENV格式镜像，布局也写入 `out/artifacts/layout.json`。这些分区由U-Boot ENV和Linux `blkdevparts` 解析，无传统MBR/GPT表。

|编号|分区|字节偏移|容量|
|---|---|---|---|
|p1|env|0|32KiB|
|p2|idblock（DDR + SPL）|32768|512KiB|
|p3|uboot|557056|256KiB|
|p4|boot（FIT: zImage + DTB）|819200|32MiB|
|p5|userdata|34373632|16MiB|
|p6|rootfs|51150848|256MiB|

U-Boot优先按env分区名找uboot，备用raw扇区设置为0x440，与557056字节一致。DT与env都指定`/dev/mmcblk1p6`作为根分区，userdata挂载p5。容量剩余部分暂不使用，也没有自动扩容。

## 写卡和首次启动

在确认正确SD卡盘符后写卡；下列命令会覆盖整张目标卡，`/dev/sdX` 必须替换为实际SD卡设备，不能填系统盘。

```sh
cd out/artifacts
sha256sum -c SHA256SUMS
sudo dd if=mohism-sd.img of=/dev/sdX bs=4M conv=fsync status=progress
```

插入卡，接调试接口SOC_UART0的TX/RX/GND（真实UART2 M1，115200 8N1，IO电平按硬件核对）。记录DDR容量/频率、SPL、U-Boot版本、SD识别和Linux启动日志。当前根用户开发镜像未设置密码，仅用于本地bringup；未启用网络服务。

U-Boot 镜像包含 SDK rkbin 提供的 DDR 固件和信任固件。USB loader 用于 Rockchip 工具的恢复与下载；CDC 接口用于调试控制台。

## 板端逐项核对

1. 测量3.3/1.8/1.35/0.9V和相机2.8/1.8/1.2V，检查启动时序、纹波、USB峰值电流及IO电压兼容。
2. `dmesg` 检查OV9281两次芯片ID识别、LSM6DSO WHO_AM_I、CIF/DPHY链接。当前SID推导地址左0x10/右0x60仍需验证。不要在驱动绑定时使用i2c工具任意写寄存器。
3. `media-ctl -p` / `v4l2-ctl --list-devices` 确认左右CIF原始节点，按实际节点修改 `/userdata/calibration.yml` 的left/right。默认 video0/video4 来自 SDK 配置，按设备身份选择节点。
4. 检查 `/sys/devices/platform/*cif*/compact_test` 为非compact、低位对齐，采集格式Y10（16位内存容器）和1280×800。当前采集支持single/multi-plane单内存平面，不支持packed RAW10。
5. 用示波器测GPIO3_D3的已配置 FSIN（当前开发请求120Hz）、两颗相机实际曝光与U3 INT1的416Hz，观察满负载下偏移和抖动。以曝光波形计算左右时差。
6. 检查 `/dev/imu0`，在静止时验证加速度模长约9.80665m/s²、陀螺接近零，记录样本间隔和flags。详见IMU ABI文档。
7. 主机 `lsusb -t` 检查USB高速480M、两个 UVC 功能和两个 CDC ACM；用 `v4l2-ctl --list-formats-ext -d /dev/videoN` 区分 640×200 YUYV 双目和 320×200 Z16 深度。分别测试单路打开、双路打开、不同启停顺序，并检查端点不足或带宽分配错误；运行host工具读参数/改参数，保存左右与16位深度，反复热插拔验证。

服务顺序：`S40mohism-usb` 创建复合设备；`S50mohism-vins` 加载IMU模块、设置FSIN PWM、按 `function_name` 查找 `uvc.stereo` / `uvc.depth` 两个输出节点并读取各自接口号，再启动用户态程序。PWM与UVC按sysfs设备身份查找。运行时相机/IMU故障会使位姿失效、约1秒后重连，并等待静止及视觉预热恢复；串口参数服务继续运行。进程启动失败/崩溃仍没有自动重启监督，需修复原因后手动restart。

## 标定

默认预览模式不要求IMU初始化。保持 `calibrated: 0`，从原始左右图采集至少15组不同距离、姿态、覆盖整个视野的清晰棋盘对，记录真实方格尺寸。

```sh
python host/mohism.py images --stereo /dev/videoN --frames 200 --output chessboard
python host/calibrate_stereo.py chessboard/stereo --columns 9 --rows 6 --square-metres 0.025 --output stereo-measured.yml
```

采集时改变棋盘距离、倾角和画面位置。标定脚本要求配对角点，输出重投影 RMS，并拒绝大于 1px 的结果。将测得K1/D1/K2/D2/R/T合并进模板；原生采集1280×800、USB320×200的尺度已在脚本中处理。

使用真实运动数据独立求取相机→机体的Rbc/tbc、IMU→机体FRD的Rbi、IMU相对相机的时间偏移。`imu_offset`加到IMU中断时间；校准时还须处理曝光中心和CIF时间戳定义。不要把左右相机外参误作相机/IMU外参。

将标定结果置于板上 `/userdata/calibration.yml`，设置 `calibrated: 1` 后重启服务：

```sh
/etc/init.d/S50mohism-vins restart
```

启动后保持设备静止至少约0.5秒供208个样本初始化。启动采样波动超阈值或机体前向几乎竖直会拒绝初始化，并滑动重试。连续3帧视觉更新成功后才报告有效位姿。跟踪中断后也需要静止恢复，主机必须依据ODOMETRY.reset_counter识别新的本地原点。

## 验收记录

记录以下指标：启动成功率、USB热插拔、参数持久化、真实曝光同步、图像/IMU丢样率、CPU/RSS/温度、连续运行帧率、多个已知距离下的深度误差、带真值轨迹的ATE/RPE与静止漂移。
