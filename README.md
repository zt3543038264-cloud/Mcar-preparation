# Mcar-preparation

RT1064 麦克纳姆轮底盘准备工程。当前版本已集成：

- ASC-AIVision_Group2 的四轮电机、编码器、麦轮逆运动学和独立轮速 PID；
- ASC 菜单的树形节点与四键交互，并按本工程功能重组菜单项；
- rt1064_imu_vofa 的 IMU660RA 读取、静止零偏标定、Mahony/Madgwick 六轴融合、四元数积分和欧拉角解算；
- rt1064_imu_vofa 的 WiFi-SPI2.0 UDP 姿态输出；
- 编码器与 IMU 定位融合，以及目标 X/Y/Yaw 的位置外环。

## 实时任务

| PIT | 周期 | 内容 |
|---|---:|---|
| CH0 | 5 ms | IMU 读取、标定和姿态融合，定位惯性预测 |
| CH1 | 10 ms | 编码器读取、定位修正、可选位置环，再执行 PWM 测试或速度闭环 |
| CH2 | 20 ms | 四键扫描 |

UDP 发送和 IMU 重新初始化在主循环执行，不阻塞上述中断。

## 菜单

IPS200 使用竖屏 240×320，按键沿用 ASC 的操作方式：

- KEY1：进入文件夹、选择参数；参数已选中时切换步进；
- KEY2：上移；参数已选中时增加；
- KEY3：返回；参数已选中时取消选择；
- KEY4：下移；参数已选中时减小。

选中数值参数后，KEY2/KEY4 按住 1 秒开始连续加减，每 80 ms 按当前步进调整一次。长按不会连续切换文件夹或开启 Run；松开后不再补一次短按。

菜单包含 PWM_Test、Drive、Encoder、Navigation、IMU、Sensor、WiFi、PID 和 Position，超过七行自动滚动。上电默认进入 Position 页面并选择位置闭环：Enable=On、OpenLoop=Off、Run=Off，目标为 (0,0,0)，不会上电自动行驶。

进入 `PWM_Test`，将 `OpenLoop` 设为 On（自动退出位置模式并关闭 Run），设置 `UL_PWM`（左前）、`UR_PWM`（右前）、`DL_PWM`（左后）、`DR_PWM`（右后），再将 `Run` 设为 On。测试单轮时其余三轮保持 0；例如 `UL_PWM=1000` 为左前正转 10% 占空比，`-1000` 为反转 10%。当前范围为 -2000～2000（由 `Motor.h` 的 `LIMIT_PWM_MIN/MAX` 设置），频率为 17 kHz；PWM 参数步进为 1、10 或 100，原菜单小数步进在这里按 1 处理。

测试链路为 `motor_test_pwm[] → motor_pwm() → DIR/PWM`，绕过逆运动学、速度 PID、死区补偿及横移启动补偿。编码器仍读取并显示，反馈不会改变 PWM。测试模式下 `Run=Off` 在下一个 10 ms 周期将四路占空比直接置零。

`Encoder` 页面每 100 ms 自动刷新，四轮均显示 `Raw`（方向校正后、滤波前的 10 ms 计数）、`Filt`（滤波后的 10 ms 计数）和 `Total`（方向校正后的累计计数），这些列均保留编码器的真实计数。下方 `cm/s` 按各轮的分辨率换算轮缘速度。`ZeroTotal` 选中后按 KEY2 清零显示累计值，操作完成自动回到 Off，不影响电机输出和 PID。底部 `PWM` 显示四轮最终软件输出，顺序为 UL、UR、DL、DR。

编码器测试先保持 `OpenLoop=On`、`Run=Off`，手动逐个转动车轮，确认只对应那一轮的 Raw/Total 变化。再用 PWM_Test 分别给单轮正、负 PWM，确认正 PWM 时该轮读数为正、负 PWM 时为负；慢速手转时 Raw 可能间歇为 0，应观察 Total。若电机正 PWM 的物理方向不正确，调整该轮 `MOTORn_FORWARD_LEVEL`；若物理方向正确但编码器符号相反，调整对应 `ENCODER_n_FORWARD_SIGN`。如累计值跳动而轮子静止，应先检查 A/B 接线和共地。

手动车体速度模式需要同时设 `Position/Enable=Off`、`PWM_Test/OpenLoop=Off`，再在 `Drive` 设置速度并开启 Run。切换模式会停止输出、清空轮速 PID，并将共享的 `Run` 设为 Off；需要重新开启 Run。

## 电机与编码器

电机输出、轮速 PID 和调试快照统一按左前、右前、左后、右后排列。驱动使用 `project/code/Motor.h` 中的当前接线定义，仅保留一套实现。

| 车轮 | 电机通道 | DIR / PWM 引脚 | 编码器模块 | A / B 引脚 |
|---|---|---|---|---|
| 左前 UL | MOTOR1 | C10 / C11 | QTIMER2_ENCODER2 | C5 / C25 |
| 右前 UR | MOTOR2 | D2 / D3 | QTIMER2_ENCODER1 | C3 / C4 |
| 左后 DL | MOTOR3 | C7 / C6 | QTIMER1_ENCODER1 | C0 / C1 |
| 右后 DR | MOTOR4 | C9 / C8 | QTIMER1_ENCODER2 | C2 / C24 |

PWM 频率为 17 kHz，当前命令限幅为 ±2000。当前本地接线中，电机 1~4 分别使用 PWM2_MODULE2_CHB、PWM2_MODULE3_CHB、PWM2_MODULE0_CHA、PWM2_MODULE1_CHA；按键为 C13/C15/C12/C14。编码器物理编号依次对应左后、右后、右前、左前，读取时转换为统一轮序，再进入滤波和 PID。

当前车轮直径为 11 cm，减速比约 2.3。UL/UR/DL 编码器为 1024 线，DR 为 512 线；底层按 A 相上升沿计数、B 相判方向，不采用四倍频。因此前三轮约为 2355.2 计数/车轮圈，DR 约为 1177.6 计数/车轮圈。轮缘速度 `cm/s = 每10ms的计数 × 100 × π × 0.11 × 100 / 该轮每圈计数`；前三轮每计数约为 1.4673 cm/s，DR 每计数约为 2.9346 cm/s。

四轮 PID 目标统一按 UL 的参考分辨率计算，PID 入口将 DR 的真实反馈乘 2；横移距离检测、四轮平均计数及闭环停止阈值也使用参考计数。Raw/Filt/Total 不乘 2，所以同速时 DR 的计数应约为其他轮的一半，页面 cm/s 应相同。减速比是近似值，最终应以各轮实测每圈计数校准；旧 PID 参数仍需在当前电机上调试。

每轮正转电平由 `MOTOR1_FORWARD_LEVEL` 至 `MOTOR4_FORWARD_LEVEL` 设置，编码器方向由 `ENCODER_1_FORWARD_SIGN` 至 `ENCODER_4_FORWARD_SIGN` 设置。当前值沿用原来各逻辑轮的方向校正；实际电机接线后的正转与反馈符号仍需在车上确认。

DR 实测反馈方向相反，已将 `ENCODER_2_FORWARD_SIGN` 修正为 +1。方向修正发生在原始计数进入滤波和累计之前，页面计数、速度及闭环反馈使用同一符号。PWM 测试中不使用反馈调整输出；相同 PWM 下应比较 cm/s，原始计数相差一倍符合 1024/512 线的硬件差异。

`tests/motor_mapping_test.c` 用记录 GPIO、PWM 和编码器调用的主机替身检查独轮正反转、初始化、限幅、反馈顺序、PID 输出通道以及 PWM 测试模式，不驱动实物。使用真实 PWM 和编码器枚举头文件，编译时将 `tests/motor_stubs`、`libraries/zf_driver` 和 `project/code` 加入头文件路径，链接 `Motor.c`、`PID.c`、`PID_config.c`、`app_control.c` 和数学库。

## IMU

IMU660RA 使用 SPI4：C23/C22/C21/C20。上电后须保持静止，连续取得 400 个合格样本后进入运行态。状态值：

- `0`：标定中；`1`：正常运行；
- `-1`：初始化失败；`-2`：采样超时；
- `-3`：采样间隔异常；`-4`：融合计算异常。

在 IMU/Recal 中置 On 可重新标定。算法由 `project/code/config.h` 的 `AHRS_METHOD` 选择。

## WiFi-SPI

当前 `config.h` 的 `IMU_WIFI_ENABLED=1`，启用逐飞 WiFi-SPI2.0 UDP 遥测。当前电机接线已移开 D12-D15，编译检查会阻止电机 DIR/PWM 退回这些引脚。WiFi 使用 SPI1：SCK=D12、MOSI=D14、MISO=D15、CS=D13、INT=B17、RST=B16；屏幕使用 SPI3、IMU 使用 SPI4。

默认发送周期为 10 ms（目标 100 帧/秒，实际受主循环及 SPI 耗时影响），上位机可运行时调整为 2~1000 ms。热点、密码和目标电脑地址在 `project/code/config.h` 配置；当前电脑端为 `192.168.0.108:8081`，模块本地端口为 `5001`。定时采样和控制在 WiFi 初始化之前启用，网络初始化失败后仍继续运行。修改上传内容无需重新烧录，重启恢复默认三通道与周期。

默认 CH0/CH1/CH2 为 roll/pitch/yaw（单位：度）。报文格式是 `N 个小端 float32 + 00 00 80 7F`，总长度为 `4*N+4` 字节，默认 16 字节。不自动添加时间戳。VOFA+ 应选择 UDP 接收和 JustFloat 解析；原来只接收 12 字节姿态数据的程序需要适配新帧长与帧尾。

上位机通过 `SUB x_cm,y_cm,nav_yaw_deg\n` 选择上传顺序，最多 40 个变量；`LIST?\n` 查询变量清单，`GET?\n` 查询当前配置，`RATE 20\n` 设置周期，`STREAM 0/1\n` 暂停/恢复上传。命令由主循环解析，数据在短临界区取一致快照，再恢复中断打包发送。未知变量、重复、空字段、超长命令或越界周期均拒绝并保留原配置。

在原“开发UDP IMU上位机GUI”的现有源码基础上，适配版本位于 [tools/wifi_spi_udp_gui](tools/wifi_spi_udp_gui/README.md)。保留三个工作区、CSV 和飞机模型，新增上传预设、变量查询和自动解析配置。默认识别 JustFloat，控制应答单独处理。使用步骤与完整协议见 [WiFi 遥测协议](docs/wifi-telemetry-protocol.md)。

WiFi 菜单显示 Status、Packets（成功数据帧数）、Attempts、LastErr、Channels、Period_ms、Stream、Commands 和 CmdErr。接口 `wifi_justfloat(...)`/`wifispi_send_floats(data,count)` 仍保留供单独调试；启用自动订阅上传时应统一使用所选通道，避免混入另一组无映射数据。

加入新的调参参数，wifispi.c里加数组

## 编码器与 IMU 定位融合

融合在 `navigation_fusion.c` 实现，由 `app_navigation.c` 接入现有调度：每个成功的 5 ms IMU 新帧预测速度，每 10 ms 读取一次方向修正后的四轮原始计数，修正速度并更新位置。重复 IMU 帧不会重复积分；编码器不会被二次读取或清零。PWM 测试和 Run=Off 时仍跟踪实际运动，融合输出已接到可选的位置外环，不包含路线回放。

坐标为起点车头 +X、左侧 +Y、从上方看逆时针航向为正。按本车实测，左转 Yaw 增大、静止 Az=+1g、IMU 的 X 朝车尾，故默认 `Mount_deg=180`、`YawFlip=Off`。去倾斜后用安装角把水平加速度映射到车体轴，再转换到固定坐标系。四轮各用自己的每圈实际计数换算距离，DR 的 512 线不会使其位移减半。位置积分使用原始增量及 IMU 航向变化，采用圆弧积分处理同时平移和旋转；不会按速度指令推算位置。

参考 [HDU 车端组合里程计](https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield) 的惯性预测、编码器修正、静止归零和打滑减权思路，以及旧 `project/code/path_follow.c` 的二维坐标变换，结合当前接口独立实现。正常编码器权重为 1，保留实际位移；速度创新与加速度差同时超限时，编码器权重短时降为 0.15，持续 8 个控制周期。此检测是启发式判断，不能识别所有打滑或消除长期漂移。参数集中在 `navigation_config.h`，需在本车重新验证。加速度预测也受安装误差、振动及 IMU 距离旋转中心的偏移影响，当前未做传感器杆臂补偿。

操作：

1. 确认后轮已换为标准 X 排列，四轮编码器正方向正确；静止上电，等待原有 IMU 标定结束，再静止约 0.5 s 建立水平加速度偏置。
2. 打开 `Navigation`，`State=2`、`Valid=1`、`Bias=1` 表示可用。`X_cm/Y_cm` 是位置，`Yaw_deg` 是相对起点的连续航向；下方显示固定坐标速度、`Slip` 和 `Rest`。根菜单有九个文件夹，自动滚动后 PID 和 Position 均可访问。
3. 停车后选中 `Zero` 按 KEY2，在下一个 10 ms 周期重建位置和航向起点，重新静止标定约 0.5 s；不重置 Encoder 页的 Total。修改 Mount_deg/YawFlip/ScaleX/ScaleY 同样重建定位起点。位置模式中这些操作使定位暂时不可用，位置控制随即关闭 Run、清轮速 PID 并停止 PWM；手动 Drive/PWM 模式仍由各自的 Run 控制。
4. State：`0` 等待 IMU，`1` 静止偏置标定，`2` 正常；`-1` 输入/配置非法，`-2` IMU 失效，`-3` 编码器异常。IMU 约 50 ms 无新帧、姿态重标定或明显异常编码器脉冲会冻结位置并使 Valid=0；排除原因后停车 Zero，不能直接继续使用旧坐标。

首次硬件验证：手推前进 50 cm，X 应增加约 50；向左推 50 cm，Y 应增加；绕车体中心旋转时航向变化、XY 应基本不变；最后测试圆弧运动。定点的实际距离取决于每圈计数及前后/横向距离比例。当前减速比仍为约 2.3，`LATERAL_CORRECTION_FACTOR` 沿用旧值，不能把菜单小数位数当成定位精度。

主机验证：运行 `tests/run_navigation_tests.ps1`，覆盖混合编码器分辨率、前后/横移、四分之一圆弧、原地旋转、倾斜重力消除、180° 安装、航向跨圈、偏置、打滑与故障冻结；并回归实际电机调度、IMU 新帧/重标定适配、Zero 和 240×320 菜单边界。

## 定点位置外环

`position_control.c` 是独立控制器。`app_control_motor_tick_10ms()` 在采集编码器并更新融合后，取得同一时刻的 X/Y/Yaw 和固定坐标速度，每 10 ms 计算一次位置指令，再调用原有 `Kinematics_Inverse()` 和 `motor_control()`。完整链路为目标点 → 融合位置反馈 → 位置外环 → 车体 Vx/Vy/Omega → 麦轮四轮目标 → 轮速 PID → PWM。PWM 测试模式不经过位置环或轮速 PID；普通 Drive 保持手动速度控制。

目标 XY 的单位是 cm，使用 Navigation/Zero 的固定坐标，不随当前车头旋转；目标 Yaw 单位是度，以 Zero 时车头为 0，逆时针为正。平移采用 `世界速度 = XY_Kp × 位置误差 − XY_Kd × 世界实测速度`，按向量长度限速；航向采用最短角误差的 P 控制并限转速。用当前融合航向 θ 转成车体指令：`Vx = cosθ × V世界X + sinθ × V世界Y`，`Vy = −sinθ × V世界X + cosθ × V世界Y`。例如车头已经左转 90°、目标仍在起点正前方时，控制器会发出车体右移指令。同时设定 XY 和 Yaw 可边平移边旋转。

默认参数在 `position_control.h`：XY_Kp=2/s，XY_Kd=0.2，Yaw_Kp=2/s；最大平移速度 20 cm/s、最大转速 1 rad/s，世界平移加速度上限 40 cm/s²、角加速度上限 2 rad/s²。平移容差 2 cm、航向容差 3°；进入容差后将指令缓降到零，实测平移速度 ≤3 cm/s 且航向变化速度 ≤0.1 rad/s，连续保持 0.2 s 才判定到达。无位置积分；容差内完成后关闭 Run，不持续锁住该点。减速比误差、IMU 漂移、打滑仍会影响真实落点，主机仿真不能替代实车调参。

菜单操作：

1. 静止上电，等 Navigation 的 State=2、Valid=1、Bias=1。需要重设起点时先停车，再 Zero 并等待定位重新就绪。
2. 上电直接显示 `Position`，默认 Enable=On、OpenLoop=Off、Run=Off；等待定位有效后设置目标，再开启 Run。手动切回位置模式时将 Enable 设 On，会自动退出 PWM 测试并关闭 Run。文件夹和参数超出七行后自动滚动。
3. 设置 TargetX_cm / TargetY_cm / TargetYaw。例如 `(50, 0, 0)` 表示向起点前方走到 50 cm，`(0, 50, 0)` 表示向左走到 50 cm，`(50, 0, 90)` 表示走向该点并左转至 90°。目标 X/Y 可为负，Yaw 在 −180° 至 +180°。
4. 首次可设 MaxV_cmps=10，其余参数先保留默认值，最后将 Position/Run 设 On。Drive 显示的是此时自动生成的 Vx/Vy/Omega，位置模式中不应在 Drive 修改速度。Position 下方显示指令和当前 XY/Yaw，ErrXY_cm / ErrYaw_deg 显示误差。
5. State=0 待启动，1 移动，2 到达范围内等待停止，3 已到达；−1 定位不可用，−2 参数非法或 PWM/位置模式冲突。到达或故障会自动关 Run、清轮速 PID、PWM 归零；新目标需要再次 Run。定位恢复不会自动续跑。切换 Enable/OpenLoop 也取消 Run；打开 PWM_Test/OpenLoop 会退出位置模式。

所有目标以最近一次 Zero 的起点为参照，完成一个目标后设置新目标不会重置坐标。Run=On 时修改目标允许平滑转向新目标，并重新计算到达等待时间；Run=Off 时修改目标不会启动电机。

代码接口是 `app_control.h` 的 `motor_position_enabled`、`motor_position_goal` 和 `motor_position_config`。切换模式先关闭 Run，再置 `motor_pwm_test_enabled=false`、`motor_position_enabled=true`，等待至少一个 10 ms 控制周期完成切换后才能打开 Run；菜单负责这一操作。主循环读取 `app_control_get_position_snapshot()` 时要在短暂关中断区内复制快照，与导航接口一致。

主机测试额外覆盖世界/车体坐标变换、跨 ±180° 转向、限速与加速度限制、前后左右和同时转向的带滞后模型收敛、低速持续到达判定、真实融合→解算→轮速 PID→PWM 接线、定位失效和运行中 Zero 停车、模式互斥、手动 Drive/PWM 回归，以及 Position 的负数编辑和全部 17 项菜单滚动。

## 行驶距离和 X/Y 比例校准

如果 Position 的 State=3，Navigation 显示约 100 cm，而地面实测只走了约 50 cm，说明定位距离偏大约两倍，位置环只是按错误的距离反馈提前到达；调整位置 Kp 不能校正这种比例误差。如果 State 仍为 1/2，却已经停住，则要先检查低速轮速闭环、死区和反馈，不应直接按比例补偿。

距离基础换算为 `每个轮子位移 = 原始计数 × π × 轮径 / 车轮每圈计数`。本工程使用 A 相上升沿 1 倍计数，车轮每圈计数暂按 `编码器线数 × 2.3`，不是 4 倍计数。若其它条件成立，显示/实际=2 意味着配置的车轮每圈计数只有实际的一半，等效减速比可能应接近 4.6；这只是现象对应的推算，不能代替实测。抬起车体，将每个车轮同方向手动转满 5~10 圈，用 Encoder/Total 增量除以圈数，直接核对每圈实际计数，能避开编码器线数/减速比口径混淆。

Y 与 X 的误差不一定相同。当前横移位移额外乘 `LATERAL_CORRECTION_FACTOR=0.901589`，逆解则除以该值；这是一组正逆配套系数，并非简单的重复修正。它来自旧底盘，应针对当前麦轮、地面和负载重新标定。横移滑动、轮子安装、实际 Yaw 偏差和各轮计数比例也会影响斜向落点，应先分别测试 (100,0) 与 (0,100)，再测试 (100,100)，各次从同一起点方向 Zero。

Navigation 的 `ScaleX/ScaleY` 范围为 0.1~5：分别额外修正车体前后和左右里程增量，先修正再按实际 Yaw 转入固定坐标。当前按实测反馈设置默认值为 `ScaleX=0.52`、`ScaleY=0.61`。ScaleY 乘在原 0.901589 横移系数上，总横移定位系数为约 0.549969。

1. 保持 Run=Off，Zero 后等待 Valid=1、Bias=1，沿地面直线手推一段已知距离，尽量保持航向不变。记录前后试验的实际距离 Lx 与 Navigation 的 X 增量 Nx；横移试验记录 Ly 与 Y 增量 Ny。
2. `ScaleX新 = ScaleX旧 × |Lx/Nx|`，`ScaleY新 = ScaleY旧 × |Ly/Ny|`。也可用自动定点后的落点实测，但分母应取实际 Navigation 读数，避免到达容差影响。若实际方向与页面符号相反，先修正方向，不能用负比例掩盖。
3. 例如旧比例均为 1，前进实际 50 cm、页面 100 cm，则 ScaleX=0.5；横移实际 60 cm、页面 100 cm，则 ScaleY=0.6。此为计算示例，不代表本车已测得这两个数。
4. 参数修改后自动重建坐标并取消 Run，重新静止等待定位就绪，再开 Run 测试。不要在运行中修改。修改仅作用于定位位移/速度，不改变 Encoder 原始计数、轮速 PID 的参考计数或 Kinematics 的轮速单位。基础每圈计数校准应优先，修正基础参数后须重新标定这两个附加比例，避免重复补偿。
5. 菜单参数重启后恢复 `navigation_config.h` 的 `NAV_FORWARD_SCALE_DEFAULT/NAV_LEFT_SCALE_DEFAULT`，当前未保存至 Flash。实测确认后将默认值写入配置再编译。

## 构建

用 Keil MDK 打开 `project/mdk/rt1064.uvprojx`，构建目标 `nor_sdram_zf_dtcm`。已使用 Arm Compiler 6.19 验证：0 errors，0 warnings。

## 上游整理分支移植记录

上游为 `zt3543038264-cloud/Mcar-preparation`，本次以 `upstream/整理` 的 `980c833` 为基线保留完整历史，移植 fork 中 `191ddb9`（位置环）的定位融合与位置控制。比较基线为 `00c49d8`（麦轮运动学解算），详细差异、接口适配和验证见 [上游版本比较与位置环移植](docs/upstream-zhengli-position-port.md)。
