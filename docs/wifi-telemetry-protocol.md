# WiFi-SPI 遥测、上传变量订阅与滑杆调参

固件：`project/code/wifispi.c`；上位机：`tools/wifi_spi_udp_gui`，根据既有 DearPyGui UDP 上位机扩展。上位机选择的是设备实际上传的变量及顺序，设备保留一个全局订阅，重启恢复默认配置。

## 接入

1. 编译、烧录当前工程。`config.h` 已启用 WiFi；SSID/密码沿用现有配置。电脑与模块连接同一网络；确认固件目标电脑 IP 是电脑当前地址，当前配置为 `192.168.0.108`。目标端口 `8081`，模块端口 `5001`。
2. 启动 `tools/wifi_spi_udp_gui/dist/WiFiSPI_UDP_MCAR.exe` 或在该目录执行 `python app.py`。监听 `0.0.0.0:8081`，点击开始监听。收到数据后，点击“模块地址取最近来源”；也可手工填写实际模块 IP 和端口 5001。
3. 在“麦轮遥测”区域选择 IMU/定位/编码器/轮速环预设，或查询变量、逐个添加，编辑上传顺序与周期，点击“应用上传配置”。上位机依次确认暂停、SUB、RATE、恢复，自动生成每个字段的小端 float32 解析规则。
4. 在实时绘图页勾选字段，CSV 保存沿用现有功能。原 IMU 固件若发送不带尾部的 12 字节数据，应取消“MCAR JustFloat 校验”。

网络尚未连到实物测试，协议单元测试和 GUI 模拟设备验收通过。固件上电仍为位置模式、Run=Off，遥测命令管理上传配置，不启动电机。WiFi 初始化只在启动时限次重试，运行中未新增自动重连。

## UDP 格式

数据包：`N 个 IEEE754 小端 float32 + 00 00 80 7F`，长度 `4*N+4`，1~40 通道。默认 3 通道依次为 `roll_deg,pitch_deg,yaw_deg`，16 字节，默认周期 10 ms。所有字段（包括状态、整数计数）都转换为 float32。Total/control_ticks 超过 16777216 后整数可能不再逐个精确表示，累计数在 MCU 内仍保留原整数。

控制请求：ASCII，区分大小写。原有 SUB/RATE/STREAM/LIST?/GET? 每条以 LF `\n` 结尾，CRLF 也可；新增滑杆包以 `]` 结束，可不带换行。逐飞 SPI 接收是字节流，本实现接受分段与连续多条命令；建议在同一个 UDP 数据报中发送一条完整命令。命令正文最多 1023 字节，滑杆包长度包含左右方括号。单个配置操作串行等待应答，不要同时用多个客户端改变订阅。

控制应答：独立 ASCII UDP 数据报，以 `MCAR ` 开头、LF 结束，与数据包区分。所有应答和遥测均发到固件预设电脑 IP:8081，**不按命令发送方的临时源端口回包**。新版上位机复用监听 socket 发送命令，源端口同为 8081，兼容限定固定对端的模块。接收端先判断帧尾与控制前缀，再校验数据长度；控制应答不进入 float 解析、曲线或 CSV。

| 请求（以下均需换行） | 成功应答 | 含义 |
| --- | --- | --- |
| `LIST?` | `MCAR LIST roll_deg,pitch_deg,...` | 当前固件全部可订阅名称，单数据报 |
| `SUB x_cm,y_cm,nav_yaw_deg` | `MCAR SUB x_cm,y_cm,nav_yaw_deg` | 原子替换通道列表，顺序即数据顺序 |
| `RATE 20` | `MCAR RATE 20` | 周期毫秒，整数 2~1000 |
| `STREAM 0` | `MCAR STREAM 0` | 暂停数据，命令接收继续运行 |
| `STREAM 1` | `MCAR STREAM 1` | 恢复数据 |
| `GET?` | 三个独立数据报：`MCAR SUB ...`、`MCAR RATE ...`、`MCAR STREAM ...` | 查询当前配置，UDP 接收顺序不限 |

失败应答为 `MCAR ERR <原因>`，例如 `unknown variable`、`duplicate variable`、`invalid channel count`。整条 SUB 验证通过才替换，失败不会局部修改。RATE/STREAM 也只在合法时生效。

运行时切换建议顺序：`STREAM 0` → 等确认 → `SUB ...` → 等确认并更新解析映射 → `RATE ...` → 等确认 → `STREAM 1` → 等确认再收数据。应用期间丢弃二进制数据；对未确认命令每秒重试，最多三次，失败显示原因并停止推进。收到错误后先读取配置或重新应用。配置尚未确认时，不应仅凭上位机输入框的内容改变解析映射。

UDP 可能丢包、重复、乱序；命令是幂等的。当前数据格式不带通道版本号、帧序号或 MCU 时间戳，同长度的极端延迟旧帧无法完全识别。切换时暂停并丢弃缓存可以减少误解析；帧率与时间轴反映上位机接收时间。网络命令在主循环执行，SPI 收发保持开中断；收命令轮询目标为 10 ms，实际响应还受主循环和模块耗时影响。

## 滑杆数据包：修改一个参数

只需要三个字段，UTF-8/ASCII 文本均可，但类型和参数名使用下面的 ASCII 名称：

```text
[slider,参数名,数值]
```

例如滑杆改变位置环 Kp：

```text
[slider,pos_xy_kp,2.5]
```

也接受带双引号的 JSON 数组，数值必须是数字，不能是字符串：

```json
["slider", "pos_xy_kp", 2.5]
```

三字段分别是控件类型 `slider`、参数名 `pos_xy_kp`、要写入的值 `2.5`。空格可以省略。以 `]` 立即结包，不需要额外发送 `\n`；同一接收流中可连续出现多包，也可和原有换行命令混用。解析仅支持这种简单数组，不支持嵌套数组、字符串转义、额外字段或任意内存地址。

成功后返回实际写入的 float32 值：

```text
MCAR SLIDER pos_xy_kp 2.5\n
```

失败返回 `MCAR ERR <原因>\n`，例如：

```text
MCAR ERR unknown parameter
MCAR ERR parameter out of range
MCAR ERR parameter requires Run off
MCAR ERR expected [slider,parameter,number]
```

每包修改一个参数，校验失败不修改。只更新 RAM，重启恢复启动配置。轮速 PID 修改会同时更新初始化参数和实际 PID，并清除该轮 PID 历史状态；重复发送同一值且运行参数已经一致时不重复清除。调参不启动电机，不改变模式。

### 已支持的参数名

| 参数名 | 范围 | 单位/作用 |
| --- | --- | --- |
| `pos_xy_kp` | 0.01～20 | 平移位置比例，1/s |
| `pos_xy_kd` | 0～5 | 平移速度阻尼系数 |
| `pos_yaw_kp` | 0.01～20 | 航向比例，1/s |
| `pos_max_speed_cmps` | 1～100 | 平移最大速度，cm/s |
| `pos_max_omega_radps` | 0.05～3 | 最大角速度，rad/s |
| `pos_max_accel_cmps2` | 1～300 | 平移最大加速度，cm/s² |
| `pos_max_alpha_radps2` | 0.05～10 | 最大角加速度，rad/s² |
| `pos_xy_tolerance_cm` | 0.5～20 | 位置到达容差，cm |
| `pos_yaw_tolerance_deg` | 0.5～20 | 航向到达容差，度 |
| `ul_kp,ul_ki,ul_kd` | 各 0～1000 | 左前轮速度 PID |
| `ur_kp,ur_ki,ur_kd` | 各 0～1000 | 右前轮速度 PID |
| `dl_kp,dl_ki,dl_kd` | 各 0～1000 | 左后轮速度 PID |
| `dr_kp,dr_ki,dr_kd` | 各 0～1000 | 右后轮速度 PID |
| `goal_x_cm,goal_y_cm` | 各 -10000～10000 | 目标坐标，cm；仅 Run=Off 可改 |
| `goal_yaw_deg` | -180～180 | 目标航向，度；仅 Run=Off 可改 |
| `scale_x,scale_y` | 各 0.01～10 | 定位比例；仅 Run=Off 可改，修改后定位会重新 Zero |

数值支持普通小数、负数和科学计数法，例如 `-125`、`2.5`、`1e-2`。NaN、Infinity、十六进制数、带引号的数值和超出范围的值不接受。实际应用值以应答为准。

新增参数在 `wifispi.c` 的 `g_slider_parameters` 中加一条映射即可，例如：

```c
SLIDER("my_gain", &my_gain, 0.0f, 10.0f, 0),
```

五项依次是上位机使用的名称、可写 `float` 变量地址、最小值、最大值、是否仅允许 Run=Off 写入。运行控制每 10 ms 从这些参数读取新值，赋值在短临界区内完成，网络收发不关中断。当前只支持 `slider` 控件类型。

上位机界面可直接将“滑杆对应的参数名”和“滑杆当前数值”组成上述数组发送到模块 IP:5001；接收 `MCAR SLIDER ...` 作为确认，不进入 JustFloat 绘图。现有 GUI 的原控制解析器尚不认识 `SLIDER` 应答，由你新增的调参界面接收处理即可。拖动时建议限频，释放滑杆时再发送最终值；UDP 无序号，固件按实际收到的顺序应用，必要时用已有同名遥测通道核对。

## 公开变量及单位

逻辑轮顺序：UL 左前、UR 右前、DL 左后、DR 右后。`ul_*`、`ur_*`、`dl_*`、`dr_*` 四组名称由 `*` 对应的后缀展开。

| 名称 | 单位/解释 |
| --- | --- |
| `roll_deg,pitch_deg,yaw_deg` | IMU 原始安装坐标姿态，度；yaw 不是 Navigation/Zero 相对航向 |
| `x_cm,y_cm,nav_yaw_deg` | Zero 固定坐标位置 cm、相对航向度；+X 为 Zero 时车头、+Y 左侧、左转正 |
| `vx_cmps,vy_cmps` | 融合固定坐标速度 cm/s |
| `ax_mps2,ay_mps2` | 固定坐标加速度 m/s² |
| `nav_status,nav_valid,bias_ready,stationary,slipping` | 定位状态及 0/1 标志；状态 -3 编码器故障、-2 IMU 故障、-1 输入错误、0 等待、1 标定、2 运行 |
| `enc_weight,enc_rejected` | 融合编码器权重、累计拒绝样本数 |
| `goal_x_cm,goal_y_cm,goal_yaw_deg` | 位置环当前目标，固定坐标 cm/度 |
| `cmd_vx_cmps,cmd_vy_cmps,cmd_omega_radps` | 喂给 Drive 的车体前向/左向 cm/s、逆时针角速度 rad/s |
| `pos_status,distance_cm,yaw_error_deg` | 位置环状态、到目标距离 cm、航向误差度；状态 -2 参数错误、-1 无定位、0 空闲、1 移动、2 收敛、3 到达 |
| `run,position_enabled,open_loop` | Run、位置模式、PWM 直驱模式，0/1 |
| `scale_x,scale_y` | 当前定位附加比例，默认 0.52/0.61；Y 另乘旧横移系数 0.901589 |
| `accel_x_g,accel_y_g,accel_z_g` | IMU 原始轴加速度 g，X 朝车尾 |
| `gyro_x_dps,gyro_y_dps,gyro_z_dps` | IMU 原始轴角速度 deg/s，未扣静止零偏 |
| `imu_status,imu_cal_percent` | IMU 状态及静止标定进度 0~100；状态定义见 imu.h |
| `ul_raw,ur_raw,dl_raw,dr_raw` | 每 10 ms 方向修正后的物理编码器增量；DR 512 线，其余 1024 线，相同转速约半数 |
| `ul_filt,ur_filt,dl_filt,dr_filt` | 物理增量滤波值，仍未作 DR ×2 归一化 |
| `ul_total,ur_total,dl_total,dr_total` | 方向修正后的累计物理计数，不受 ScaleX/Y 影响 |
| `ul_cmps,ur_cmps,dl_cmps,dr_cmps` | 按各轮线数换算的轮速 cm/s，不受定位 ScaleX/Y 影响 |
| `ul_target,ur_target,dl_target,dr_target` | 轮速环目标参考计数/10 ms，与 DR 原始计数不能直接一比一比较 |
| `ul_pwm,ur_pwm,dl_pwm,dr_pwm` | 最终有符号 PWM，含闭环补偿或 PWM 测试实际输出 |
| `control_ticks` | 电机控制累计 10 ms tick 数 |

扩展变量在 `wifispi.c` 的 `TELEMETRY_VARIABLES(X)` 中增加一项，名称、ID 和快照取值由同一表生成。上位机下次查询 LIST 即可发现；配置输入是允许变量名称清单，不解析任意 C 表达式或内存地址。
