# USB / MAVLink 协议 v0.4

## USB

一个 USB2 High Speed 复合设备内包含两个 UVC 视频功能和两个 CDC ACM 串口。所有接口共用一个 USB 地址和一对 VID/PID。默认开发标识为 `1d6b:0104`，设备版本 `0x0400`；量产需使用已分配标识。

|功能实例|控制/数据接口|图像格式|每帧字节数|
|---|---|---|---|
|`uvc.stereo`|0/1|640×200 YUYV，左右各 320×200|256000|
|`uvc.depth`|2/3|320×200 Z16|128000|
|`acm.usb0`|4/5|MAVLink|—|
|`acm.usb1`|6/7|调试控制台|—|

接口顺序由 configfs 链接顺序确定，应用启动后读取实际 `bInterfaceNumber`，不把第二路的控制请求误当作第一路。板端通过 `/sys/class/video4linux/video*/function_name` 区分两个输出节点。主机必须检查视频捕获能力、USB 接口和格式，不能依赖固定 `/dev/videoN` 编号；驱动还可能创建额外元数据节点。

双目每行 1280 字节：前 640 字节为左灰度图 `[gray,128]`，后 640 字节为右灰度图 `[gray,128]`。普通视频查看器可显示左右并排图像。深度不在 YUYV 图像内；独立 Z16 每行 640 字节，每个像素为小端 uint16 毫米值，GUID 的 UVC 字节表示（首四字节 `Z16 `；后十二字节 `00 00 10 00 80 00 00 aa 00 38 9b 71`）。禁止对深度做 RGB 转换或有损编码。

双目支持请求 15/30/60fps；深度 USB 频率为 `min(USB_FPS, DEPTH_FPS)`，可单独打开、关闭。深度 0 表示无效，计算范围 100–65534mm。未标定、禁用或无法计算深度时输出零深度。正常处理时只发布新计算的深度，不用未计算帧覆盖它；USB 可重复最近结果，超过 200ms 未发布则清零。故障后的已排队缓冲仍可能短暂包含旧数据。

标定前为原始双目预览；标定后为校正图。两个流独立调度，没有共同曝光时间/帧序号元数据，主机收到时间只是到达时间；即使帧率相同也不能按主机序号配对。主机工具分别保存两个目录和 `arrival.csv`。

HS 双目端点每微帧最多 2048 字节，深度端点最多 1024 字节。双目 60fps + 深度 30fps 纯数据 19.2MB/s；双 60fps 为 23.04MB/s。两 UVC 和两 ACM 当前需要 8 个非零 IN 端点、2 个 OUT 端点。主机工具使用 Linux Z16 捕获路径。不支持按当前配置在 Full Speed 下传输图像。

## MAVLink

MAVLink common，发送 MAVLink2，可解析 MAVLink1/2。设备端 `/dev/ttyGS0`，主机通常 `/dev/ttyACM0`，须按 USB 接口 4 确认；第二个串口 `/dev/ttyGS1` 专用于内核日志、登录与 Shell，见 [USB 调试串口](usb-debug.md)。波特率标记 115200，CDC USB 实际传输速率由 USB 决定。system=1，component=197（VISUAL_INERTIAL_ODOMETRY）。接受本机目标或广播目标。

支持 HEARTBEAT（约1Hz）、PARAM_REQUEST_LIST、PARAM_REQUEST_READ（名称或索引）、PARAM_SET、PARAM_VALUE、ODOMETRY。所有参数以 REAL32 传输；错误类型、NaN、越界或只读写入均回传当前值。写入磁盘失败也会回滚。

|参数|权限|默认|范围/作用|
|---|---|---|---|
|VIO_ENABLE|读写|1|0/1，禁用时清空估计并保持预览；重新启用需要静止或运动初始化|
|SYNC_MAX_MS|读写|2|0.1–20ms，实际还受标定配置 `max_skew` 的上限约束|
|VIO_VALID|只读|0|启用输出且最近200ms内估计有效|
|FW_VERSION|只读|500|版本编码 major×10000+minor×100+patch，即0.5.0|
|ACC_NOISE|读写|0.1|0.001–10，加速度白噪声标准差密度，m/s²/√Hz|
|GYRO_NOISE|读写|0.01|0.00001–1，角速度白噪声标准差密度，rad/s/√Hz|
|ACC_BIAS_RW|读写|0.0001|0.000001–0.1，加速度偏置随机游走密度，m/s²/√s|
|GYRO_BIAS_RW|读写|0.00001|0.0000001–0.1，角速度偏置随机游走密度，rad/s/√s|

读写参数保存在 `/userdata/mohism-params.yml`，通过临时文件/fsync/rename 更新，重启加载。噪声参数在下一图像周期生效。这些默认值是工程起点，不是测得的传感器噪声。

ODOMETRY 每个新的有效估计时间戳发送一次，实际随处理率；重复轮询不复制消息，串口背压时保留最新未发送估计；时间戳为板端 CLOCK_MONOTONIC 微秒。父坐标 LOCAL_FRD：初始位置为原点，重力方向为下，初始机体前向投影为水平前向；子坐标 BODY_FRD。位置为世界坐标，线速度转换到机体坐标，四元数为 `[w,x,y,z]`、body→world。必须正确标定 Rbi/Rbc，不能把任意 IMU 原始轴直接称为 FRD。

每次故障恢复或禁用/启用都会建立新的本地原点，进程内 `reset_counter` 按模256递增，主机必须按重置段处理轨迹；进程重启后计数从0开始。静止或运动初始化及连续3次联合求解预热完成前不发送有效ODOMETRY。

当前对外协方差和角速度以 NaN 表示未知，quality=-1。尚未实现 TIMESYNC、命令式重置、MAVLink Signing、参数分组扩展或标定文件上传。重启/重新标定需通过板端 SD/调试串口操作。

0.5.0 的后端改为紧耦合滑窗。旧参数文件中的 `POSE_POS_SD`、`POSE_ROT_SD` 不再生效，不会被解释为偏置随机游走；升级后需读取新参数并重新配置。`ACC_NOISE`、`GYRO_NOISE` 是连续噪声密度，投影噪声通过 YAML 的 `pixel_noise` 配置。回环修正也可能增加 `reset_counter`；本地优化窗口不会因地图修正重置。
