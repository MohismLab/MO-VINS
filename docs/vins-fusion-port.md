# VINS-Fusion 算法移植与健壮性验证（0.2.0）

日期：2026-10-28。上游为 HKUST-Aerial-Robotics/VINS-Fusion，固定提交
`be55a937a57436548ddfb1bd324bc1e9a9e828e0`，完整源码位于忽略的 `vendor/VINS-Fusion/`。
`sh scripts/bootstrap.sh` 可复现拉取；`sources.lock.json` 记录来源。
移植部分保留 GPL-3.0 版权、作者和修改声明，见根目录 LICENSE 及
`third_party/VINS-Fusion/NOTICE.md`。镜像包含许可证；打包同时输出包含本项目源码、
构建脚本和固定版本 MAVLink 头文件的 `mohism-vins-source.tar.gz`，SDK 由脚本按锁定版本获取。

## 移植范围

|上游组件|本项目实现|实际使用方式|
|---|---|---|
|`featureTracker/feature_tracker.cpp`|`src/feature_tracker.cpp`|保留 ID/跟踪寿命；优先保留长寿命特征，8px 空间掩膜补点；时域和左右双向 LK，回投误差≤0.5px|
|同上左右匹配|稀疏双目三角化|校正后行误差≤0.75px，视差≥0.75px，深度0.1–15m；PnP 使用这些稀疏深度，不再读取整数毫米 SGBM 深度|
|`factor/integration_base.h`|`src/preintegration.cpp`|两端加速度/角速度的中点积分、15状态雅可比、18维噪声注入、偏置重传播；去掉 ROS/Eigen/Ceres，使用 OpenCV 和 SO(3) 指数|
|上游缓冲重传播|`Preintegration::repropagate`|最多4096步，可用于偏置更新后的重积分；当前 ESKF 按相邻采样使用单步预积分传播，不是滑窗 IMU 因子优化|

上游的非线性滑窗求解、IMUFactor/投影因子联合优化、边缘化、特征逆深度优化、
在线外参/时间偏移估计、回环及位姿图**没有移植**。当前后端仍是松耦合15状态 ESKF。
没有将项目重命名为完整 VINS-Fusion，也没有将它的精度或实时性结论转用于本板。
上游 `Estimator::failureDetection()` 在锁定版本开头直接 `return false`，因此本项目
另外实现了实际生效的检查和恢复流程，没有直接依赖这个函数。

## 本项目补充的检查

- 重用 SGBM 实例，并增加左右一致性检查；遮挡、超范围和不一致深度为0。
- 连续跟踪：至少12个 PnP 内点、至少50%一致性，RANSAC 后使用 LM 重投影细化；
  再检查正深度、1.5px重投影、当前双目深度一致性和内点凸包面积≥图像2%。
- 位姿信息矩阵条件数>1e8时拒绝；从几何雅可比估算6维观测协方差，叠加参数指定的
  位置/姿态方差下限。此估计条件于上一帧三维点，没有完整传播深度不确定性及跨帧相关性。
- IMU噪声参数为连续密度：两个端点的独立噪声方差取 `2*sigma²/dt`，偏置随机游走取
  `sigma_bias²/dt`，使单步方差按dt增长。相邻区间共用原始端点的相关性未完整建模。
- 滤波使用 Cholesky 求解、创新卡方门限22.46、Joseph协方差更新、误差注入后重置雅可比和
  对称化。视觉观测无效、重复/过期、非有限值或非SO(3)旋转均拒绝。
- 修正很小旋转的 logSO3，避免 OpenCV 的 matrix→Rodrigues 小角舍零影响偏置雅可比。
- 每个图像时刻对真实相邻 IMU 样本线性插值，不把未来采样值直接向过去保持。
- 初始化要求208个连续样本（约0.5秒）、采样间隔≤10ms、静止方差/重力模长与陀螺均值检查。
  失败后以16点滑动重试，初始化会清除旧状态、协方差和观测时刻。
- 非有限数值、重复/倒退时间戳、IMU间隔>50ms、接近传感器量程
  （加速度模长≥19.5m/s²或角速度模长≥4.3rad/s）使本轮估计失效。
- 视觉帧间隔>200ms、平移速度>8m/s、角速度>4rad/s拒绝；滤波速度>30m/s、加计偏置>2.5m/s²、
  陀螺偏置>1rad/s拒绝。这些是当前工程的支持范围，并非硬件性能承诺。
- 初始化、IMU队列和预积分历史均有上限；采集每轮最多取4帧并保留最新帧，避免积压旧图。
  MAVLink每次poll最多处理2048字节，防止连续输入完全占用主循环。

## 恢复和对外语义

`src/estimator.cpp` 为硬件独立的恢复控制器，板端和离线回放共用。

1. 等待静止 IMU 初始化，预览左右原图、深度为0，禁止有效位姿。
2. 初始化视觉，连续3个成功视觉更新后报告有效位姿；预热期间可能已有有效深度。
3. 遮挡、光流/PnP/创新拒绝、时间戳异常、数据过载或设备断流，立即清除旧估计并增加重置计数。
4. 继续提供预览/串口参数服务，等待再次静止及视觉预热。相机/IMU打开或读取失败会约1秒后重试。

每次重置生成新的本地 FRD 原点；**不保证跟踪中断后的轨迹连续**。
`ODOMETRY.reset_counter` 为进程内模256计数，接收端必须识别它，禁止跨epoch直接拼接位置。
进程重启后计数重新从0开始。未开始发送的旧epoch/失效/过期ODOMETRY从待发队列清除；
已经部分写入或进入USB链路的旧报文无法撤回，接收端也必须依据时间戳判断新鲜度。
`VIO_ENABLE=0` 清空估计并保持预览；重新启用会重新等待静止初始化。
过期、未来时间、非法旋转或非有限位姿不发送ODOMETRY。对外协方差和quality仍标记未知，
内部近似协方差没有经过实测统计校准。

运行时可恢复传感器故障；串口/UVC初始化失败、配置错误或整个进程崩溃仍需外部恢复。
目前启动脚本没有进程监督器，USB真实热插拔恢复也未验证。

## 自动化验证

```sh
sh scripts/build.sh host
sh scripts/build.sh sanitize
```

- `core-test`：原有尺度、平移方向、静止惯导、观测异常和时间间隔测试。
- `robustness-test`：解析加速度斜坡、旋转重力补偿、所有6个偏置雅可比列的数值差分、
  偏置重传播一致性、协方差半正定/对称、NaN、非法标定和重复时刻、持续特征ID、
  完全遮挡后失效、运动中拒绝重新初始化、静止恢复、禁用/启用和IMU时间倒退。
- `trajectory-test`：30帧三深度层场景，平移和三轴旋转、左右轻微曝光差与亮度变化、
  临时20%视野遮挡，位置误差<2cm、旋转误差<0.02rad的合成回归门限。
  这是短序列软件回归，不能解释为实际传感器精度指标。
- `replay`：生成65帧EuRoC格式的CSV/PNG序列，检查遮挡后恢复、无效输出NaN、
  重置计数、全无纹理时返回失败退出码。
- `odometry-test`：PTY上检查ODOMETRY重置计数、坐标系、禁用及过期/未来/NaN/非法旋转拒绝。
- 原有Python测试继续检查真实MAVLink字节、8参数、持久化和16位深度字节保真。

主机日志在 `out/validation/vins-fusion-host-tests.log`。
ASan/UBSan日志在 `out/validation/vins-fusion-sanitizers.log`；
ARM/Buildroot日志在 `out/validation/vins-fusion-arm-build.log`。

Sanitizer入口使用非PIE主机可执行文件。一次默认PIE构建在core进程启动阶段挂起，
已终止并保留 `out/validation/vins-fusion-sanitizers-initial-hang.log`，根因尚未确认；
此轮不计通过。非PIE ASan/UBSan构建的5项测试通过，CTest设置30秒超时防止无限等待。

## 实际数据回放入口

主机构建生成 `out/host/mohism-replay`，输入采用EuRoC目录格式：

```text
mav0/cam0/data.csv        # timestamp_ns,filename
mav0/cam0/data/*.png
mav0/cam1/data.csv
mav0/cam1/data/*.png
mav0/imu0/data.csv        # timestamp_ns,wx,wy,wz,ax,ay,az
```

角速度单位rad/s，加速度m/s²，时间戳为整数纳秒。以独立测量的对应数据集标定文件运行：

```sh
docker run --rm -u "$(id -u):$(id -g)" -v "$PWD:/work" -w /work \
  mohism-build:bookworm out/host/mohism-replay measured.yml recordings/mav0 out/trajectory.csv
```

图像尺寸必须匹配capture_width/height；配置仍使用本机固定320×200处理尺寸。
正确填入K/D、双目R/T和相机/IMU/FRD变换，不能直接复用本板示例标定到EuRoC。
CSV按每帧输出valid、重置计数、位置/速度/旋转向量、内点数、处理耗时和状态；
无效位置/速度/旋转为NaN。没有任何有效帧时退出码为3，解析错误为1。
回放工具目前不读取真值、不自动计算ATE/RPE；需要独立对齐并按重置段分别评估，同时报告
有效覆盖率和失效次数，不能只挑正常片段。

## 尚未完成的验收

没有运行真实EuRoC/TUM-VI或本板带真值运动数据；没有真实ATE/RPE、长时间漂移、
弱纹理/动态场景/强震动/快速转动覆盖率，也没有RV1106 CPU/RSS、帧率或热稳定性实测。
没有滑窗紧耦合与回环，全景被遮挡后需要静止恢复，错误标定仍会产生系统误差。
因此目前完成的是可复现的关键组件移植及故障防护，不能宣称算法已经“完全健壮”。
