# MOHISM RV1106G3 双目 VIO 固件

本目录包含基于当前嘉立创 **RV1106G3-VINS** 原理图的板级适配、Buildroot 工程、`/dev/imu0` 内核驱动、双目深度/视觉惯性算法、USB UVC + CDC ACM 和 MAVLink 参数服务。

**当前是已构建的软件原型，尚未完成真板验收。** 原理图 DRC 返回未通过；相机地址、IO 电压、FSIN/IRQ 时序、DDR 启动及实测标定仍需验证。不能将生成镜像理解为已验证可直接交付的 VINS 产品。完整核查见 [硬件报告](docs/hardware-review.md)。

构建产物在 `out/artifacts/`：`mohism-sd.img`、`boot.img`、`rootfs.img`、`uboot.img`、DDR/SPL idblock 与 USB 下载 loader、`mohism_imu.ko`、ARM 可执行文件及 `SHA256SUMS`。镜像使用 Rockchip SDK 的 ENV 分区格式，**不是 GPT/FAT 格式**。不要用通用分区工具修改已生成镜像。

## 构建

需要本机 Docker。脚本在容器内以当前用户构建，工程映射到 `/work`，避免中文和空格路径影响上游构建工具。现有 AppImage 环境中的 `LD_LIBRARY_PATH`/`PYTHONHOME` 会导致系统 Git/Python 错误，脚本会清除这些变量。

```sh
sh scripts/bootstrap.sh
sh scripts/build.sh environment
sh scripts/build.sh all
```

可分别使用 `host`、`kernel`、`uboot`、`rootfs`、`package`。修改用户态源码后重建本地 Buildroot 包时，先在同一容器内执行 `make ... mohism-vins-dirclean`，避免上游 local-package 的旧 stamp 保留旧源码。详细流程见 [构建与上板](docs/bringup.md)。源仓库和固定提交在 `sources.lock.json`。

## USB 使用

U-Boot 提供独立 USB CDC 调试控制台，进入 Linux 时重新枚举；早期 SPL/DDR 日志使用 UART2。连接方式与验证限制见 [USB 调试串口](docs/usb-debug.md)。

USB 是一个物理复合设备，具有 UVC 和两个 CDC ACM 接口。Linux 主机通常得到 `/dev/ttyACM0`（MAVLink）、`/dev/ttyACM1`（调试日志与 Shell）和一个 UVC `/dev/videoN`；设备号会随其他设备变化。`lsusb` 列出物理设备，`lsusb -t` 显示接口驱动。

```sh
python3 -m venv .venv
. .venv/bin/activate
pip install -r host/requirements.txt
python host/mohism.py --serial /dev/ttyACM0 params
python host/mohism.py --serial /dev/ttyACM0 set ACC_NOISE 0.15
python host/mohism.py --serial /dev/ttyACM0 odometry
python host/mohism.py images --video /dev/videoN --output capture --frames 100
```

UVC 首版固定为 **960×200 YUYV，15fps**：三块 320×200 分别传左灰度、右灰度、16 位毫米深度。深度占第三块的全部原始字节，**不能经过 YUYV→RGB 转换或有损压缩**；主机工具关闭颜色转换，保存左右 8 位 PNG 与深度 16 位 PNG。详见 [USB 与 MAVLink 协议](docs/protocol.md)。这是一条自定义打包的标准 UVC 视频流，不是三个独立摄像头节点。

## 标定和算法

默认 `calibrated: 0`：提供原始左右预览、MAVLink 控制和全零无效深度，不输出 ODOMETRY。传感器打不开时控制服务仍可工作，图像保持黑色。标定模板中的数字都是示例，不能作为测量结果。

用原始预览采集不同角度/距离的棋盘图像，可运行 `host/calibrate_stereo.py` 求双目参数。相机到 IMU 的旋转/平移、IMU 到机体 FRD 的轴变换、相机与 IMU 的时间偏移还需独立标定；全部验证后将配置写入板端 `/userdata/calibration.yml` 并重启服务。

算法已移植 VINS-Fusion 的持续特征跟踪、双向光流/双目检查和 IMU 中点预积分（含雅可比、协方差及偏置重传播），后端为 15 维误差状态滤波。加入重投影/深度/退化检查、近似观测协方差、严格时间插值及故障自动重新初始化；跟踪丢失或 IMU 断流后禁止有效位姿，保持静止可恢复，新的本地原点通过 MAVLink `reset_counter` 标识。

仍是松耦合原型，没有滑窗联合优化、边缘化、回环、重定位或 NPU 加速；连续 VO 误差相关性没有完整建模。未测量目标板帧率、真实轨迹误差、深度精度或热稳定性。移植来源、回归测试及 EuRoC 格式回放方式见 [算法移植与验证](docs/vins-fusion-port.md)。用户态算法按 GPL-3.0 发布，上游版权和修改说明见 [第三方声明](third_party/VINS-Fusion/NOTICE.md)。

[IMU 驱动 ABI](docs/imu-driver.md) · [硬件核对与阻断项](docs/hardware-review.md) · [实机验收步骤](docs/bringup.md)
