# USB / MAVLink 协议 v0.3

## USB

一个 USB2 High Speed 复合设备；先 UVC（控制接口 0、视频流接口 1），后两个 CDC ACM（MAVLink 控制/数据接口 2/3，调试控制/数据接口 4/5）。开发默认使用 Linux gadget 示例 VID/PID `1d6b:0104`，量产需替换为已分配的标识。USB 60fps 的纯帧数据带宽为 23.04MB/s，包含端点开销后仍须真机确认。

每个 UVC 帧 384000 字节，960×200，YUYV，启动配置选择 15/30/60fps。每行 1920 字节：

|行内字节|含义|
|---|---|
|0–639|320 个左灰度像素，每个 `[gray,128]`|
|640–1279|320 个右灰度像素，每个 `[gray,128]`|
|1280–1919|320 个深度值，逐像素 uint16 little endian，单位 mm|

深度 0 表示无效；稠密深度可单独限频，未计算的新帧整幅深度为零，不复用旧深度；范围 100–65534mm，超范围/无匹配输出 0。标定前为原始双目预览和全零深度；标定后为校正后的双目图与深度。检查 `VIO_VALID`/标定配置判断状态，不要用图像颜色推断深度有效性。普通视频查看器可以看左右部分，但其 RGB 结果不能恢复深度。

当前 UVC 流没有自定义曝光时间/帧序号元数据，USB 重复帧和延迟须在上板时测量；不能直接把主机收到视频的时间当作曝光时间用于再次融合。

## MAVLink

MAVLink common，发送 MAVLink2，可解析 MAVLink1/2。设备端 `/dev/ttyGS0`，主机通常 `/dev/ttyACM0`，须按 USB 接口 2 确认；第二个串口 `/dev/ttyGS1` 专用于内核日志、登录与 Shell，见 [USB 调试串口](usb-debug.md)。波特率标记 115200，CDC USB 实际传输速率由 USB 决定。system=1，component=197（VISUAL_INERTIAL_ODOMETRY）。接受本机目标或广播目标。

支持 HEARTBEAT（约1Hz）、PARAM_REQUEST_LIST、PARAM_REQUEST_READ（名称或索引）、PARAM_SET、PARAM_VALUE、ODOMETRY。所有参数以 REAL32 传输；错误类型、NaN、越界或只读写入均回传当前值。写入磁盘失败也会回滚。

|参数|权限|默认|范围/作用|
|---|---|---|---|
|VIO_ENABLE|读写|1|0/1，禁用时清空估计并保持预览；重新启用需要静止初始化|
|SYNC_MAX_MS|读写|2|0.1–20ms，实际还受标定配置 `max_skew` 的上限约束|
|VIO_VALID|只读|0|启用输出且最近200ms内估计有效|
|FW_VERSION|只读|300|版本编码 major×10000+minor×100+patch，即0.3.0|
|ACC_NOISE|读写|0.1|0.001–10，加速度白噪声标准差密度，m/s²/√Hz|
|GYRO_NOISE|读写|0.01|0.00001–1，角速度白噪声标准差密度，rad/s/√Hz|
|POSE_POS_SD|读写|0.03|0.001–10m，视觉位置观测标准差下限|
|POSE_ROT_SD|读写|0.02|0.0001–1rad，视觉姿态观测标准差下限|

读写参数保存在 `/userdata/mohism-params.yml`，通过临时文件/fsync/rename 更新，重启加载。噪声参数在下一图像周期生效。这些默认值是工程起点，不是测得的传感器噪声。

ODOMETRY 每个新的有效估计时间戳发送一次，实际随处理率；重复轮询不复制消息，串口背压时保留最新未发送估计；时间戳为板端 CLOCK_MONOTONIC 微秒。父坐标 LOCAL_FRD：初始位置为原点，重力方向为下，初始机体前向投影为水平前向；子坐标 BODY_FRD。位置为世界坐标，线速度转换到机体坐标，四元数为 `[w,x,y,z]`、body→world。必须正确标定 Rbi/Rbc，不能把任意 IMU 原始轴直接称为 FRD。

每次故障恢复或禁用/启用都会建立新的本地原点，进程内 `reset_counter` 按模256递增，主机必须按重置段处理轨迹；进程重启后计数从0开始。静止初始化和连续3帧视觉预热完成前不发送有效ODOMETRY。

当前对外协方差和角速度以 NaN 表示未知，quality=-1；不虚构质量分数。尚未实现 TIMESYNC、命令式重置、MAVLink Signing、参数分组扩展或标定文件上传。重启/重新标定需通过板端 SD/调试串口操作。
