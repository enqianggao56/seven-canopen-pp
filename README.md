# 七轴 CANopen PP 绝对定位终端

通过终端对本次初始化的在线节点提交同步绝对定位。每次 `pp-move` 重新读取七轴 JSON，只在命令执行时触发。到位后保持使能、PP 模式和周期反馈。默认启动仍提供失能参数测试，设置 `CANOPEN_PP_MOTION=1` 开放运动命令。

## 运行

运行所需的两份 EDS、四份设备手册和 CiA 402 协议 PDF 均可在 [vendor 目录](vendor/) 中查阅。

需要 Python 3.11+。现场使用 SocketCAN 的 `can0`，接口须已按 1 Mbit/s 启用。可通过 `ip -details link show can0` 查看实际波特率和链路状态。

```bash
python3.11 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m canopen_pp.main
```

| 命令 | 行为 |
|---|---|
| `init` | 逐节点进入 Pre-operational，探测身份和状态，配置并读回 PDO 映射。SDO 无响应的节点标为 OFFLINE。对通过校验的节点发送 NMT Start，启动 SYNC 并确认同一周期的 TPDO1、TPDO2。参数测试模式还要求所有在线节点均已失能。 |
| `move-test` | 确认在线节点失能，按每个在线节点三帧 RPDO 提交参数，检查同一 SYNC 周期的失能反馈。 |
| `enable` | 运动模式下读取本次 JSON 力矩，向在线节点分三批发送 0006h、0007h、000Fh，并逐阶段确认状态。 |
| `pp-move` | 重新读取定位参数，同批触发在线节点，检查其中节点 1–3 的 ACK，等待所有参与节点到位。 |
| `disable` | 对在线节点同批发送对应厂商的失能控制字，等待每个节点的新 TPDO1 确认 Switch On Disabled。 |
| `status` | 列出所有配置节点的 ONLINE/OFFLINE 标记和两组 ID，并显示在线节点的状态字、CiA 402 状态、6061 模式、机械与原始位置及速度。机械量保留三位小数。 |
| `exit`、EOF、Ctrl-C | 有轴通电或反馈不确定时先尝试整组失能，验证后关闭 SYNC 和 CAN。无法确认时输出 `Disable not confirmed` 及节点状态。 |

至少一个节点通过初始化即可运行。全部离线时，终端报告初始化失败，仍可查看 `status` 或重新 `init`。收到错误身份、异常 SDO 响应、PDO 校验错误或 Fault 时，初始化报错。

本次在线名单从初始化持续到下次扫描。节点恢复连接后，先确认当前在线组已安全失能，再执行 `init` 重扫并核对名单。若在线节点在运动中丢失 TPDO，控制器锁定运动，并向本次在线组执行失能。

CAN 后端、通道、波特率和主站周期在配置区集中设置。默认 `sync_period_ms=100`，设置为 `50` 即使用 50 ms。主站发送标准帧 `080h`、DLC 0。程序不访问从站的 `1005h/1006h`。只有显式定位命令产生 RPDO3 的 Bit 4 上升沿。替换 CAN 接口时，后端需能区分本机发送回显并提供报文时间戳。[配置文件](canopen_pp/config.py)

## 七轴绝对位置配置

执行 `enable`、`pp-move` 或 `move-test` 时读取仓库根目录的 `config/pp_targets.json`。顶层需要整数 `version = 1`、非空字符串 `name`、`position_mode = "absolute"` 及对象 `axes`。节点键必须恰好为 `"1"` 至 `"7"`，文件中的排列顺序任意。随附的数值仅供失能通信验证，实际运动的机械安全目标需另行确认。[完整配置](config/pp_targets.json)

| 节点 | 必填字段 | 工程单位 |
|---|---|---|
| 1–3 | position、velocity、acceleration、deceleration | mm、mm/s、mm/s²、mm/s² |
| 4–6 | 上述四项及 torque_per_mille | deg、deg/s、deg/s²、deg/s²、‰ |
| 7 | 上述五项 | mm、mm/s、mm/s²、mm/s²、‰ |

加载器检查完整七轴结构的未知字段、缺项和重复键。终端对本次在线节点检查数值类型、有限性、机械设计限值及可编码范围。三个轮廓幅值必须为正且可编码，力矩为 1–1000 的整数。读取失败或内容无效时，只拒绝本次命令，保留连接和稳定使能状态。冻结的七轴数据供本次调用使用，运动中编辑文件只影响下一次读取。[配置加载器](canopen_pp/pp_config.py)

## 重复绝对定位

**节点 2 承载条件**　运动异常和主动关闭都可能对 Kinco 发送 Disable Voltage。在上电调试前，落实独立防坠支撑，按 iSMD 手册 §5.7.2 现场验证抱闸、驱动器停机设置、制动延时、负载保持和外部急停。当前 TPDO 没有机械抱闸闭合反馈。历史失能记录只确认了电气状态。[Kinco 手册](vendor/iSMD高性能低压伺服一体机使用手册20260303.pdf)

```bash
CANOPEN_PP_MOTION=1 .venv/bin/python -m canopen_pp.main
```

1. 输入 `init`，核对在线名单、实际位置和速度。七轴机械零点为 0、方向为 +1。根据实际位置在 JSON 中填写本次绝对目标。
2. 输入 `enable`，确认在线节点均为 Operation Enabled、6061h = 1。
3. 输入 `pp-move`。完成后使用 `status` 查看在线节点反馈。连接、NMT Operational 和默认 100 ms 的 SYNC 持续运行。
4. 在另一终端修改并保存同一 JSON，再输入 `pp-move`。每次按机械行程验证绝对目标，按各轴设计上限验证速度、加速度和减速度。一次命令可以直接定位到行程内的远端目标。
5. 定位完成后可继续执行步骤 4，结束时输入 `disable`、`exit`。命令同步等待定位完成，运动中的 Ctrl-C 进入异常停机和退出处理。

| 节点 | 到位位置误差 |
|---|---:|
| 1–3 | 0.02 mm |
| 4–6 | 0.05° |
| 7 | 0.005 mm |

运动完成时，主站用表中阈值核对实际位置误差。TPDO2 持续更新实际速度，供 `status` 查看。例如节点 2 从 0 mm 定位到 JSON 中的 100 mm 时，通过预检后会执行完整的 100 mm 行程。[轴配置](canopen_pp/config.py)

**运动握手**　主站先确认所有在线节点反馈有效、处于 PP 模式和 Operation Enabled 状态，并确认节点 1–3 的 ACK 为 0。已有成功定位时还检查上次目标的保持状态。唯一的 SYNC 线程先向在线节点发送 RPDO1、RPDO2 和 bit 4 为零的 RPDO3，再发 SYNC。收到该装载 SYNC 周期的全部在线节点完整 TPDO 后，下一批 RPDO3 统一发送 001Fh，随后发送一个 SYNC。节点 1–3 的新反馈 bit 12 全为 1 后，向在线节点同批发 000Fh，再等待节点 1–3 的 bit 12 全部复位。节点 4–7 跳过 bit 12 检查。以上各阶段均检查反馈时效和状态。传输类型 1、逐帧 TX 回显、独立装载周期和后续 TPDO 构成协议级同步屏障，当前反馈映射未逐项回读驱动器内部的轮廓寄存器。[运动控制](canopen_pp/motion.py)

到位确认要求清位批次之后两个不同 SYNC 周期的全部在线节点完整反馈均显示实际位置误差在窗口内。节点 1–3 还须满足 bit 10 = 1、ACK = 0。节点 4–7 按实际位置确认本次到位，并在下次命令前按实际位置复查上次目标的保持状态。运动时限按各轴距离、速度和两段斜率估算三角形或梯形轮廓，取最长时间并加入量化、SYNC 和稳定裕量。相同目标也完成独立握手，起点已在窗口内时会提示无需位移。

**异常停机与恢复**　Fault、跟随误差、模式异常、反馈失效、节点 1–3 的 ACK 握手失败、发送失败或到位超时均锁住后续定位。异常处理沿用已知的厂商失能帧整批覆盖本次在线节点，检查新的失能反馈。部分发送失败后，调度器抑制普通 SYNC，恢复批次先覆盖全部控制字。缺少失能确认会报告节点状态，须使用现场安全回路。故障原因排除、确认负载受支撑后，手动执行 `disable` 确认失能、`enable` 恢复。程序不复位驱动器故障，也不重发上升沿。成功到位直接返回命令行。[控制器](canopen_pp/controller.py)

现场验收应记录连续两次不同目标的 CAN 抓包，包括各次装载、在线节点同批上升沿、节点 1–3 的 ACK 高低、到位位置与速度、保持期间状态及最终失能。两次之间只修改 JSON。固件同步生效时刻、实际起动偏差、轮廓和力矩效果以及垂直轴抱闸仍需实测。下文的历史抓包为失能参数测试记录。

## 发送一次失能测试批次

启动终端后依次输入以下命令。等待 `init` 显示在线节点均已失能后执行 `move-test`。保留终端输出，供逐帧比对。

```text
init
move-test
status
exit
```

1. `move-test` 复查快照，使用 `PPEncoder.encode()` 编码每个在线节点。全部成功后按现有映射为每个节点打包三帧。RPDO1 的小端 `<iI` 对应有符号绝对位置和无符号速度，RPDO2 的 `<II` 对应无符号加速度和减速度，二者 DLC 均为 8。
2. RPDO3 由各驱动器的 `control_data(disable_word)` 生成。节点 1–3 为 `00 00 01`（DLC 3），节点 4–7 为 `01 00 01 00 00`（DLC 5）。两组模式字节均为 1，控制字分别为 0000h、0001h。RobStride 的配置力矩显示在报告中，失能帧中的目标力矩恒为 0。
3. 主线程预检失能状态及两路反馈。唯一的 SYNC 线程在实际发送前再次检查上一周期所有在线节点的反馈，依次发送在线组的 RPDO1、RPDO2、RPDO3。调度器逐帧记录 TX echo 时间戳，收到末帧回显后发送本批次的 080h（DLC 0）。
4. 对该次 SYNC 检查所有在线节点的 TPDO1、TPDO2 及状态字，逐节点打印工程输入、编码整数、CAN-ID、DLC、十六进制数据和回显时间。丢失该周期的反馈或出现非失能状态均报告失败。后续周期按原有 100 ms 调度继续发送 SYNC。
5. 发送异常时，报告仅统计已获得回显的帧。缺少回显的帧在物理总线上的结果未知。调度器抑制后续空闲 SYNC，可通过 `disable` 或退出时的安全处理发送完整失能批次；参数不会自动补发。进入 Operational 后没有 PP 对象的 SDO 回读。[批次实现](canopen_pp/controller.py) · [报告](canopen_pp/move_test.py)

## SocketCAN 抓包核对

1. 以下按七轴全部在线举例，部分在线时按本次名单核对帧数和 ID。本阶段使用主站 SocketCAN 接口 `can0` 验收。在主站另开一个终端，确认接口已按 1 Mbit/s 启用，然后在输入 `init` 前开始抓包。

   ```bash
   ip -details link show can0
   candump -L can0 > pp-can0.log
   ```

2. 主站执行一次 `init` 和一次 `move-test`。观察后续几个 SYNC 周期再输入 `exit`，在监听端按 Ctrl-C 结束记录。以下命令提取三组 RPDO 和 SYNC，原始文件保留 TPDO 和其他流量。

   ```bash
   grep -E ' (20[1-7]|30[1-7]|40[1-7]|080)#' pp-can0.log
   ```

3. 找到 `201#` 开始的批次，逐帧与本次终端报告对照。检查 CAN-ID、DLC（每两个十六进制字符代表一个字节）、全部数据字节及顺序。分别按小端有符号位置和无符号轮廓值解码，与该轴四个编码整数核对。确认两种 RPDO3 的失能控制字、PP 模式及节点 4–7 的零力矩。
4. 按时间戳确认 21 帧中间没有 SYNC，407h 之后紧跟批次 SYNC，随后有七轴两路 TPDO。检查整段记录只出现一次参数批次，后续普通 SYNC 继续运行。确认七轴状态字均为 Switch On Disabled 后，将本次抓包核对结果记入实机记录。

终端的 `TRANSMISSION COMPLETED` 表示本批获得发送回显并收齐失能反馈。抓包核对使用同一次运行保存的 candump 日志。

### 历史失能实机记录

使用随附 JSON 在主站 `can0` 执行了一次 `init → move-test → exit`。主站接口的整段记录中仅有一组 21 帧 RPDO，逐帧核对了 ID、长度及数据。首帧到批次 SYNC 为 32.906 ms，末帧到 SYNC 为 1.990 ms，之后记录到 49 个普通 SYNC。批次 SYNC 后收齐 14 路反馈，节点 1–3 的状态字为 `4270h`，节点 4–7 为 `0040h`，全部为 Switch On Disabled。下列原始摘录从批次前一帧 SYNC 延续到批次后的下一帧 SYNC。[主站 can0 抓包摘录](docs/pp-test-can0.log)

随后操作者提供了第二次运行的完整抓包，共 9348 帧。其中第 4504 至 4540 行包含唯一一批 21 帧 RPDO，按 CAN-ID、DLC 及全部数据字节与随附 JSON 的编码结果逐帧相符。该批次中间没有 SYNC。第 4540 行的 `407#0100010000` 与第 4541 行的 `080#` 相隔 1.964 ms。批次前后均收齐七轴两路反馈，状态字分别为 `4270h`（节点 1–3）和 `0040h`（节点 4–7）。其后还有 83 个普通 SYNC，整段记录无重复参数批次。操作者确认抓包使用运行控制器的主站适配器 `can0`。**本阶段七轴失能 PP 参数下发已通过 can0 总线验收。** [第二次运行的主站抓包](https://github.com/enqianggao56/seven-canopen-pp/blob/bd0eb0552ded363f16751d5d64e7f805d0f38f3a/pp-independent.log#L4504-L4541)

## 离线机械坐标转换

`MechanicalConverter` 只依赖 Python 标准库。目标位置用 mm 或 deg，原始位置用有符号 32 位 inc。七轴参数集中在 `MECHANICAL_CONFIG` 中，也可通过 `MechanicalConverter(configs)` 注入按节点 ID 索引的配置。[机械配置](canopen_pp/config.py) · [七轴行程及参考轴](vendor/readme.md)

```python
from canopen_pp.mechanical import MechanicalConverter

converter = MechanicalConverter()
converter.get_config(1).unit           # "mm"
converter.validate_target(1, 600.0)    # 通过时返回 None
converter.target_to_raw(1, 600.0)      # 3276800 inc
converter.raw_to_actual(1, 3276800)    # 600.0 mm
converter.raw_to_actual(1, 6619136)    # 1212.0 mm，保留超出目标行程的反馈
converter.raw_to_velocity(1, 65536)    # 0.732421875 mm/s，iSMD 手册比例
converter.raw_to_velocity(4, 100)      # 60.0 deg/s
converter.raw_to_velocity(7, 300)      # 1.0 mm/s，丝杠螺母直线速度
```

K 是参考轴每转的机械位移，C 是同一转对应的**通信位置增量**。目标按闭区间检查，例如节点 1 允许 0 至 1200 mm，输入 1300 mm 会抛出 `TargetSoftLimitError`。通过检查后计算 `zero_offset + direction × target × C / K`。工程数值按十进制表示计算，最终原始坐标取最近整数，恰为半个计数时向远离零的方向舍入。结果超出 I32 抛出 `RawPositionOverflowError`。

反馈只检查输入的整数类型及 I32 范围，再计算 `(raw − zero_offset) × direction × K / C`。节点 6 保留完整圈数，节点 7 的 mm 表示丝杠螺母直线位移。软件目标限位用于校验输入，现场安全保护仍依靠驱动器限位、机械限位开关及急停。

`zero_offset` 是机械零点对应的原始计数，`direction` 为 +1 或 −1。默认 0 和 +1 只是初始配置，实际机械零点和方向尚待现场确认。Kinco 的 `0x6063` 和 RobStride 的 `0x6064` 已按 I32 解码。项目采用的 C 分别为每电机轴转 65536 inc、每输出法兰转 16384 inc。各实轴的目标及反馈计数与实际位移是否匹配仍待实机验证。

### 速度反馈与终端读数

下表是离线反馈示例。两列原始值保留驱动器上报的有符号整数。位置和速度共用 K、单位及方向，零点偏移只作用于位置。

```text
Node  Motor  Statusword  CiA402 State        6061  Position    Raw(inc)  Velocity      Vel(raw)
1     Kinco  0x4637      Operation Enabled   1     600.000 mm  3276800   0.732 mm/s    65536
4     RS06   0x0037      Operation Enabled   1     45.000 deg  2048      60.000 deg/s  100
7     EL05   0x0037      Operation Enabled   1     10.000 mm   81920     1.000 mm/s    300
```

- **Kinco** 使用 `velocity_unit="kinco_606c_dec"`。iSMD 手册 §7.2 对 `606Ch` 给出 `raw = rpm × 512 × 编码器分辨率 / 1875`。按项目电机轴 C = 65536 和手册分辨率 65536 换算，机械速度为 `raw × direction × K / (C × 16.384)`。配置中的 **16.384 秒**是由 `60 × 512 / 1875` 得到的等效计数时间基准，与 SYNC 和反馈刷新周期独立。对于已确认直接上报 inc/s 的设备，可单轴配置 `"inc_per_second"`（1 秒基准）。例如节点 1 的 65536 在此配置下显示 `12.000 mm/s`。[iSMD 手册 §7.2](vendor/iSMD高性能低压伺服一体机使用手册20260303.pdf) · [速度单位配置](canopen_pp/config.py)
- **RobStride** 使用 `velocity_unit="rpm_0_1"`。三款手册 §5.4 均标注 `606Ch` 每计数为 0.1 rpm，机械速度为 `raw × 0.1 × direction × K / 60`。项目按输出法兰作为速度参考轴换算，`606Ch` 条目未单独说明参考轴，现场对应关系仍待验证。换算直接使用项目的 K。[RS06 手册](vendor/RS06使用说明书260428.pdf) · [RS00 手册](vendor/RS00使用说明书260428.pdf) · [EL05 手册](vendor/EL05使用说明书2600428.pdf)

TPDO1 超时时，`Position`、`Raw(inc)` 和模式显示 `--`，状态列标记 `TPDO1 STALE`，状态字保留最后一次有效读数。TPDO2 超时时，`Velocity` 显示 `--`，`Vel(raw)` 显示 `TPDO2 STALE`。两路独立更新。有效反馈的某个物理量转换失败时，对应列显示 `CONV ERROR`，原始整数仍可读，表后列出节点、字段及原因。

## 离线 PP 参数编码（阶段 01）

`PPEncoder.encode()` 接受一个轴的绝对目标和三个正的轮廓幅值，返回四个对象字典整数。可在只有 Python 标准库的环境中运行。下面的节点 4 示例中，负角度决定目标位置的符号，加速和减速可以分别指定。[编码器](canopen_pp/pp.py)

```python
from canopen_pp.pp import PPCommand, PPEncoder

command = PPCommand(node_id=4, target_position=-45, velocity=60, acceleration=12, deceleration=3)
params = PPEncoder().encode(command)
# PPParameters(target_position=-2048, profile_velocity=100,
#              profile_acceleration=20, profile_deceleration=5)
```

| 节点 | 目标闭区间 | 速度上限 | 加速度上限 | 减速度上限 |
|---|---|---:|---:|---:|
| 1、2、3 | 分别为 0–1200、0–1100、0–700 mm | 600 mm/s | 600 mm/s² | 600 mm/s² |
| 4、5、6 | 分别为 ±90、±45、±360° | 720 °/s | 720 °/s² | 720 °/s² |
| 7 | 0–20 mm | 14 mm/s | 14 mm/s² | 14 mm/s² |

这些数值用于机械设计限值检查，实机调试速度另行选定。编码器只接受 1–7 的整数节点和有限的 `int`、`float` 工程量，拒绝布尔值。目标由 `MechanicalConverter.target_to_raw()` 检查行程、方向、零点及 I32 范围。若非零目标量化后与机械零点重合，编码器也会拒绝。位置对象为 I32（−2³¹ 至 2³¹−1），三项轮廓对象均为 U32（0 至 2³²−1）。两份 EDS 的这四个条目均未给出更窄的取值范围。[设计限值](canopen_pp/config.py) · [Kinco EDS](vendor/motor.eds) · [RobStride EDS](vendor/RS06.eds)

### 七轴离线编码结果

以下为默认机械配置下 `PPEncoder.encode()` 的十进制输出。速度 v、加速度 a、减速度 d 分别取各轴的设计上限。直线轴的 v 用 mm/s，a、d 用 mm/s²。旋转轴的 v 用 °/s，a、d 用 °/s²。节点 4–6 取负向行程端点，以核对绝对位置的符号。[七轴边界测试](tests/test_pp.py)

| 节点 | 驱动器 | 绝对目标 | 输入 v / a / d | 607Ah 位置（I32） | 6081h 速度（U32） | 6083h 加速度（U32） | 6084h 减速度（U32） |
|---:|---|---:|---|---:|---:|---:|---:|
| 1 | Kinco iSMD | 1200 mm | 600 / 600 / 600 | 6553600 | 53687091 | 53687 | 53687 |
| 2 | Kinco iSMD | 1100 mm | 600 / 600 / 600 | 6007467 | 53687091 | 53687 | 53687 |
| 3 | Kinco iSMD | 700 mm | 600 / 600 / 600 | 4828968 | 67815273 | 67815 | 67815 |
| 4 | RS06 | −90° | 720 / 720 / 720 | −4096 | 1200 | 1200 | 1200 |
| 5 | RS00 | −45° | 720 / 720 / 720 | −2048 | 1200 | 1200 | 1200 |
| 6 | EL05 | −360° | 720 / 720 / 720 | −16384 | 1200 | 1200 | 1200 |
| 7 | EL05 | 20 mm | 14 / 14 / 14 | 163840 | 4200 | 4200 | 4200 |

### 厂商依据与换算规则

设 K 为参考轴每转对应的机械位移，v 为速度，a、d 分别为加速度和减速度。下表给出量化前的计数。

| 节点及项目参考轴 | 6081h | 6083h | 6084h |
|---|---|---|---|
| 1–3，Kinco 电机轴，编码器分辨率 E = 65536 | (v / K) × 60 × 512 × E / 1875 | (a / K) × 65536 × E / 4000000 | (d / K) × 65536 × E / 4000000 |
| 4–7，RobStride 输出法兰 | (v / K) × 600 | (a / K) × 600 | (d / K) × 600 |

Kinco §7.3 对 `6081h` 明列 rpm 到 DEC 的比例，对 `6083h/6084h` 明列 rev/s² 到 DEC 的比例。表 8–2 指定编码器分辨率，表 9–11 给出 CANopen 样例，200 rpm 编为 `0x00369D03`，100 rev/s² 编为 `0x0001A36E`。编码路径独立于 `606Ch` 的反馈配置。[iSMD 手册](vendor/iSMD高性能低压伺服一体机使用手册20260303.pdf)

RobStride 三款手册 §5.6 给出 `6081h` 每计数 0.1 rpm、`6083h` 每计数 0.1 rpm/s。CiA 402 §12.3.9 规定可选对象 `6084h` 与 `6083h` 同单位。项目共用的 RS06 EDS 将 `6084h` 声明为可写、可映射的 U32，据此编码为 0.1 rpm/s。三款手册的 CANopen PP 表均未单列该对象，也未逐对象标注参考轴。离线计算采用项目机械表定义的输出法兰，节点 7 使用每法兰转 2 mm。固件的实际参考轴仍待实机验收。[七轴机械表](vendor/readme.md) · [RS06 §5.6](vendor/RS06使用说明书260428.pdf) · [RS00 §5.6](vendor/RS00使用说明书260428.pdf) · [EL05 §5.6](vendor/EL05使用说明书2600428.pdf) · [CiA 402 §12.3.9](vendor/CanOpen-DS402协议-电机驱动器.pdf)

**现场配置记录**　现场已通过上位机将节点 4、5、6、7 的 `acc_status` 全部设为 `1`，四个节点的 PP 轮廓减速度均可独立于加速度配置。通过 CANopen 写入 `6084h` 后的实际减速响应仍待实机验收。

手册未规定主站如何舍入任意小数输入。本实现对三项轮廓幅值分别检查设计限值，按输入的十进制表示精确换算，然后向下取整。因此在设计上限处也不会因量化而增大幅值。换算值超过 U32、取整后为零或任一字段无效时，整个调用抛出 `ValueError` 或其子类。返回值只表示离线编码成功。

## 每轴的五组 PDO

所有 PDO 使用经典 CAN、标准标识符、小端布局和传输类型 1。表中 `n` 是节点号。[各字段字节偏移](vendor/readme.md)

| 槽 | 通信 / 映射 | CAN-ID | 映射字，依字节顺序 | DLC |
|---|---|---|---|---|
| RPDO1 | 1400 / 1600 | 200h+n | 607A0020、60810020 | 8 |
| RPDO2 | 1401 / 1601 | 300h+n | 60830020、60840020 | 8 |
| RPDO3（Kinco，1–3） | 1402 / 1602 | 400h+n | 60400010、60600008 | 3 |
| RPDO3（RobStride，4–7） | 1402 / 1602 | 400h+n | 60400010、60600008、60710010 | 5 |
| TPDO1 | 1800 / 1A00 | 180h+n | 60410010、60630020（Kinco）或 60640020（RobStride）、60610008 | 7 |
| TPDO2 | 1801 / 1A01 | 280h+n | 606C0020 | 4 |

七轴的 TPDO 反馈周期现已统一。每个 SYNC 周期，每个节点各发送一帧 TPDO1 和一帧 TPDO2，七轴合计 14 帧。这样可以减少 Kinco 节点在同一周期内的重复更新和不必要的 CAN 总线负载。Kinco 与 RobStride 按相同频率刷新反馈，主站可按同一 SYNC 周期核对七轴状态，减少状态判断的时序歧义，为下一阶段 PP 实机运动测试提供基础。

三款 RobStride 手册 §5.6 均将 `6071h Target_torque` 列为 PP 运动前的必设参数，在该模式下它限定最大力矩绝对值。字段为小端 I16，单位为额定力矩的千分之一。节点 4–7 分别使用本次 JSON 的 `torque_per_mille`，随附配置为 100‰。失能及失败回退时，`0001h` 控制字随帧写入力矩值 0，下次使能从 Shutdown 阶段恢复配置值。手册还要求运动前设置轮廓速度和加速度。[配置](canopen_pp/config.py) · [RS06 手册](vendor/RS06使用说明书260428.pdf) · [RS00 手册](vendor/RS00使用说明书260428.pdf) · [EL05 手册](vendor/EL05使用说明书2600428.pdf)

初始化按 ID 选择设备描述。节点 1–3 使用 Kinco EDS。节点 4–7 使用随项目提供的 RS06 EDS，RS00 和 EL05 的映射逐台通过 SDO 验证。初始化读取厂家号及当前状态字，检查 EDS 的对象类型、可映射标记和读写属性。需要重映射时先置 COB-ID 的禁用位，再清零数量、写条目和数量、配置传输类型，最后启用。若额外启用的 RPDO 可改写该节点的任一 RPDO3 字段（包括 RobStride 的力矩上限），初始化禁用该额外槽并读回。在线节点配置结束后再次读回每个在线节点五个槽的 COB-ID、传输类型、数量和每个有效条目。SDO Abort 或读回不符会给出节点及对象地址。配置中断时，已变更的映射可能保留在从站 RAM，原因排除后重新执行 `init`。[Kinco EDS](vendor/motor.eds) · [RobStride EDS](vendor/RS06.eds)

进入 Operational 后，程序仅使用同步 PDO 和 SYNC。接收线程记录每次主站 SYNC 回显和两路 TPDO 的时间戳。长度错误使对应反馈失效。使能各阶段需要同一批次 SYNC 或更晚的一组完整反馈，状态通过掩码解析。终端等待输入期间接收及 SYNC 调度继续运行。各 RPDO 按发送回显排序，最后一帧回显后直接发送 SYNC。USB、操作系统调度及 CAN 仲裁会产生实际间隔。在 Kinco 节点 1，前期测试曾观察到类型 1 的控制字在 SYNC 前已影响状态，因此同批报文及后续 TPDO 确认不能证明七个功率级在同一物理时刻动作。[调度器](canopen_pp/communication.py) · [状态机](canopen_pp/controller.py)

## 常规使能模式的状态转换与失败处理

对于 Ready、Switched On、Quick Stop Active 或部分已使能的组合，`enable` 先将整组归一到 Switch On Disabled，再逐阶段使能。全部已使能且处于 PP 时重复 `enable` 只确认反馈。Fault、Fault Reaction Active、未知状态、失效反馈会中断使能。中断后输出回退前各轴状态并尝试整组失能。批次传输中断会抑制空闲 SYNC，恢复时必须先以完整的失能批次覆盖待处理控制字。

Kinco 使用 Disable Voltage（`0000h`），RS06、RS00 和 EL05 使用厂家手册 §5.3 的 `0001h`。失能确认目标均为 Switch On Disabled。软件反馈只确认驱动器状态机，涉及承载和安全断电的操作依靠现场机械支撑、抱闸及外部安全回路。[CiA 402](vendor/CanOpen-DS402协议-电机驱动器.pdf) · [Kinco 手册](vendor/iSMD高性能低压伺服一体机使用手册20260303.pdf) · [RS06 手册](vendor/RS06使用说明书260428.pdf) · [RS00 手册](vendor/RS00使用说明书260428.pdf) · [EL05 手册](vendor/EL05使用说明书2600428.pdf)

## 测试和实机记录

```bash
.venv/bin/python -m pytest -q
```

默认测试使用 python-can 虚拟总线，模拟零至七个 canopen LocalNode SDO 服务，检查正常顺序、映射读回、SDO Abort、部分发送失败、故障及回退、反馈超时、模式和失能确认。新增的配置及批次用例也在默认测试中运行。历史使能实机用例在参数测试模式下跳过。PP 运动回归使用虚拟总线，覆盖连续两次改参、整批触发、保持使能及异常锁定。真实定位由操作者使用上文的终端命令验收。

此前使用三字节 RPDO3 布局时，已在 `can0`、1 Mbit/s、100 ms 主站周期先验证 RS06 节点 4，再对七轴做整组验证。整组测试中节点 5–7 初始为 Quick Stop Active。经失能归一后得到以下 TPDO1 实测值。

| 节点 | Shutdown 后 | Switch On 后 | Enable Operation 后 | 使能时 6061 | 最终失能后 |
|---|---|---|---|---|---|
| 1–3，Kinco | 0x4231 | 0x4233 | 0x4637 | 1 | 0x4270 |
| 4–7，RS06 / RS00 / EL05 | 0x0031 | 0x0033 | 0x0037 | 1 | 0x0040 |

独立 SocketCAN 监听器确认每阶段七帧 RPDO3 之后紧接同一帧 SYNC，三次使能阶段观察到的末帧至 SYNC 间隔分别约为 0.52、1.05、1.03 ms。抓包也检查了 NMT Start 后的 SDO 与 RPDO1/2 发送数均为零。七轴第一次初始化曾在 NMT 发送回显等待 300 ms 时超时。初始化 NMT 改用 1 s 回显等待后，实机回归和正式终端流程均通过。故障注入在虚拟总线上完成。[实机回归用例](tests/test_hardware.py)

另以 50 ms 周期执行了一次实机初始化，连续 12 个后续周期均收齐七轴两路 TPDO。独立监听器测得 SYNC 间隔最小 47.803 ms、中位数 50.183 ms、最大 52.227 ms，七轴全程回报 Switch On Disabled。50 ms 下的使能时序及长期抖动尚未实测。

采用含 6071h 的五字节 RPDO3 后，已在同一条实机总线上初始化节点 4–7。独立监听器从四台驱动器的 SDO 应答逐一核实了 `1602:00 = 3`，映射项依次为 `60400010`、`60600008`、`60710010`。四台驱动器的 TPDO1 均回报 `0040h`（Switch On Disabled）。五字节使能帧和失能帧已在虚拟总线验证，本次实机测试范围为映射及反馈。
