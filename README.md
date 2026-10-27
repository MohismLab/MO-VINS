# MOHISM VINS

MOHISM VINS 是一款双目视觉惯性设备。设备在板端计算深度和位姿，通过 USB 向笔记本提供图像、MAVLink 参数配置和调试终端。

仓库包含板级设备树、Buildroot 构建工程、IMU 内核驱动、视觉惯性算法、USB 复合设备配置和主机工具。

目前硬件尚未生产。软件已完成交叉编译、主机回归测试和镜像离线检查，启动、同步、帧率和精度仍需实板验证。

## 硬件与系统

| 项目 | 配置 |
| --- | --- |
| 双目相机 | 双路单色相机，原生分辨率 1280×800 |
| IMU | SPI 通信，数据就绪中断，驱动节点 `/dev/imu0` |
| USB | 一路 USB 2.0，工作在 Device 模式 |
| 系统 | Linux 5.10.160（PREEMPT_RT）+ Buildroot 2023.02.6 |
| 启动介质 | 当前工程使用 SD 卡 |

默认构建启用 PREEMPT_RT，IMU 的硬中断入口记录时间戳，SPI 读取在线程中执行。实时补丁已完成 ARM 构建验证，实际调度延迟仍需上板测量。构建和测试方法见 [实时内核](docs/realtime.md)。

双目相机已配置共用 FSIN 外触发。IMU INT1 用于数据就绪中断和时间戳，不能代替相机与 IMU 的共同硬件同步；共同同步电路和驱动仍待完成。当前原理图的 DRC 也尚未通过，电平、相机地址及启动时序需要继续核对。详见 [硬件核对](docs/hardware-review.md) 和 [同步方案](docs/hardware-sync.md)。

## USB 接口

连接笔记本后，固件将 USB 配置为一个复合设备，包含以下接口：

| 接口 | 用途 | 板端节点 |
| --- | --- | --- |
| UVC | 左右图像和深度数据 | 启动脚本查找对应的视频节点 |
| CDC ACM | MAVLink 参数读写、状态和里程计 | `/dev/ttyGS0` |
| CDC ACM | Linux 日志和 Shell | `/dev/ttyGS1` |

Linux 主机上的串口通常为 `/dev/ttyACM0` 和 `/dev/ttyACM1`，视频节点为 `/dev/videoN`。编号会随连接顺序变化，使用前应确认接口身份。`lsusb` 显示一个物理设备，`lsusb -t` 可查看各接口的驱动。

U-Boot 也提供 USB CDC 调试控制台，进入 Linux 时 USB 会断开并重新枚举。DDR、SPL 等早期启动日志仍需通过 UART2 查看。连接方式见 [USB 调试串口](docs/usb-debug.md)。当前调试镜像的 root 用户未设置密码，发布前需要配置登录认证。

### 图像格式

当前 UVC 输出为 960×200 YUYV，支持请求 15、30 或 60 fps。每帧分为三个 320×200 区域，依次存放左灰度图、右灰度图和 16 位毫米深度。

这是自定义数据打包格式，需要使用仓库内的主机工具解包。深度区域使用原始字节保存数值，经过 YUYV 转 RGB 或有损压缩会破坏深度数据。工具将左右图保存为 8 位 PNG，深度保存为 16 位 PNG。

没有计算深度的帧，其深度区域为零。深度频率与图像输出频率独立，因此并非每个 USB 帧都包含有效深度。具体布局和 MAVLink 消息见 [通信协议](docs/protocol.md)。

### 主机工具

以下示例使用 Linux 主机；请将串口和视频节点替换为实际设备。

```sh
python3 -m venv .venv
. .venv/bin/activate
pip install -r host/requirements.txt

# 读取和修改参数
python host/mohism.py --serial /dev/ttyACM0 params
python host/mohism.py --serial /dev/ttyACM0 set ACC_NOISE 0.15

# 接收状态和里程计
python host/mohism.py --serial /dev/ttyACM0 odometry

# 保存左右图和深度
python host/mohism.py images --video /dev/videoN --output capture --frames 100
```

## 算法

前端从 VINS-Fusion 移植了持续特征跟踪、双向光流检查和双目匹配，IMU 部分实现中点预积分、雅可比、协方差和偏置重传播。后端采用 15 维误差状态滤波器融合视觉与惯性数据。

当前算法是松耦合实现，尚未包含 VINS-Fusion 的紧耦合滑窗优化、边缘化、回环和重定位。代码具备时间、数值和几何检查，以及跟踪丢失、传感器断流后的重新初始化逻辑；恢复后本地坐标原点可能变化，主机应读取 MAVLink `reset_counter`。

默认配置为 `calibrated: 0`，此时只提供原始双目预览和 MAVLink 控制，深度为零，不发送有效里程计。配置模板中的内外参是示例值，使用前必须替换为实测标定结果。

双目标定可使用 `host/calibrate_stereo.py`。相机与 IMU 的外参、轴向变换和时间偏移需要另外标定。完成后将配置放到板端 `/userdata/calibration.yml`，再启用标定状态并重启服务。步骤见 [构建与上板](docs/bringup.md)。

算法来源、合成回归测试和数据回放说明见 [算法移植记录](docs/vins-fusion-port.md)。目前还没有实板轨迹误差、深度精度或长期运行数据。

## 帧率与性能

开发配置位于板端 `/etc/mohism/performance.conf`：

```sh
CAPTURE_FPS=120
USB_FPS=60
DEPTH_FPS=30
```

以上分别是双目采集、USB 输出和稠密深度的请求频率，不代表实测性能。相机驱动选择 SDK 中已有的 120 fps 模式；最终帧率还取决于外触发、曝光、双路采集、内存带宽和处理负载。

采集和处理使用有界队列，处理落后时保留最新帧。稀疏视觉跟踪与稠密深度分别调度，MAVLink 随新的有效估计发送。运行统计每 5 秒写入 `/run/mohism-status.json`。

USB 60 fps 的当前打包数据量约为 23.04 MB/s，已接近配置的等时端点预算，需要验证双串口并发时的完整率。原生分辨率的高帧率双目图像不能直接通过当前 USB 链路无压缩输出。计算依据和统计口径见 [性能说明](docs/performance.md)。

## 构建

构建需要 Docker，以及下载依赖所需的网络访问。SDK、MAVLink 和 VINS-Fusion 的版本固定在 [sources.lock.json](sources.lock.json)。

在仓库根目录执行：

```sh
sh scripts/bootstrap.sh
sh scripts/build.sh environment
sh scripts/build.sh all
```

构建在容器内以当前用户运行，仓库挂载到 `/work`，支持宿主机路径包含中文和空格。脚本会清除影响工具运行的 AppImage 环境变量。

也可以分步运行：

| 命令 | 内容 |
| --- | --- |
| `sh scripts/build.sh host` | 主机编译和回归测试 |
| `sh scripts/build.sh sanitize` | ASan / UBSan 检查 |
| `sh scripts/build.sh kernel` | PREEMPT_RT 内核、设备树和 IMU 驱动 |
| `sh scripts/build.sh kernel-standard` | 单独构建普通内核，用于回退比较 |
| `sh scripts/build.sh uboot` | U-Boot |
| `sh scripts/build.sh rootfs` | Buildroot 根文件系统 |
| `sh scripts/build.sh refresh-rootfs` | 修改应用源码后清理本地包并重建根文件系统 |
| `sh scripts/build.sh package` | 打包镜像并执行离线检查 |

修改用户态源码后，使用 `refresh-rootfs`，再执行 `package`，避免 Buildroot 复用旧的本地包。

### 构建产物

产物位于 `out/artifacts/`，该目录不提交到 Git。

| 文件 | 内容 |
| --- | --- |
| `mohism-sd.img` | 完整 SD 卡镜像 |
| `boot.img` | 内核与设备树 FIT 镜像 |
| `uboot.img` | U-Boot 镜像 |
| `rootfs.img` | 根文件系统 |
| `mohism_imu.ko` | IMU 内核模块 |
| `mohism-vins` | ARM 用户态程序 |
| `mohism-vins-source.tar.gz` | 对应源码归档 |
| `SHA256SUMS` | 产物校验值 |

目录中还包含 DDR/SPL idblock 和 USB 下载 loader。SD 镜像采用 ENV 与固定偏移布局，没有传统 MBR/GPT 分区表。写卡和恢复流程请按 [上板文档](docs/bringup.md) 操作。

## 后续计划

无线视频计划在板端将图像硬件编码为 H.264，通过 SDIO Wi-Fi 向笔记本发送压缩预览，保留现有 USB 接口。无线模块的选型、驱动、供电和接线尚未验证；当前工程还没有接入 Wi-Fi 或 H.264 发送功能。深度数值将与预览视频分开传输。

近期工作包括共同硬件同步改版、首板启动、相机与 IMU 标定，以及 USB 并发、实际帧率和算法精度测试。完整待办见 [TASKS.md](TASKS.md)，交付要求见 [产品交付基线](docs/product-delivery.md)。

## 文档与许可证

- [构建、写卡与首次启动](docs/bringup.md)
- [硬件核对](docs/hardware-review.md)
- [相机与 IMU 同步](docs/hardware-sync.md)
- [IMU 驱动接口](docs/imu-driver.md)
- [USB 与 MAVLink 协议](docs/protocol.md)
- [USB 调试串口](docs/usb-debug.md)
- [算法移植记录](docs/vins-fusion-port.md)
- [性能说明](docs/performance.md)
- [实时内核](docs/realtime.md)

本仓库用户态程序采用 [GPL-3.0](LICENSE)。内核驱动和板级文件的许可见各文件声明，上游代码保留原有许可证。VINS-Fusion 的版权与移植说明见 [第三方声明](third_party/VINS-Fusion/NOTICE.md)。
