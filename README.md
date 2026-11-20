# 七轴 CANopen 同步 PDO 使能终端

通过终端初始化七个驱动器、切换 CiA 402 状态并查看真实反馈。启动终端后由操作者输入 `init`，驱动器只在收到 `enable` 命令后进入使能流程。

## 运行

仓库包含运行所需的两份 EDS。文中引用的厂家手册和协议 PDF 为本地参考资料，可将相应文件放在 `vendor/` 下查阅。

需要 Python 3.11+。现场使用 SocketCAN 的 `can0`，接口须已按 1 Mbit/s 启用。可通过 `ip -details link show can0` 查看实际波特率和链路状态。

```bash
python3.11 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m canopen_pp.main
```

| 命令 | 行为 |
|---|---|
| `init` | 打开 CAN，将每个节点置于 Pre-operational，配置和读回全部映射，逐节点发送 NMT Start，再启动主站 SYNC。七轴同一周期的 TPDO1、TPDO2 均有效后报告成功。 |
| `enable` | 同步 RPDO3 分三批发送 `06 00 01`、`07 00 01`、`0F 00 01`。每批均等待七轴的 TPDO1 确认。末批还要求七个模式显示均为 1。 |
| `disable` | 同批发送厂商对应的失能控制字，等待七个新 TPDO1 均回报 Switch On Disabled。 |
| `status` | 显示各节点的最新状态字、CiA 402 状态、6061 模式、实际位置（inc）及实际速度（606C 原始单位）。两路 TPDO 单独判定超时。标记 STALE 的状态字是最后一次有效读数。 |
| `exit`、EOF、Ctrl-C | 有轴通电或反馈不确定时先尝试整组失能，验证后关闭 SYNC 和 CAN。无法确认时输出 `Disable not confirmed` 及节点状态。 |

CAN 后端、通道、波特率和主站周期在配置区集中设置。默认 `sync_period_ms=100`，设置为 `50` 即使用 50 ms。主站发送标准帧 `080h`、DLC 0。程序不访问从站的 `1005h/1006h`。本阶段不发送位置目标、轮廓参数、Bit 4 触发或 Heartbeat。替换 CAN 接口时，后端需能区分本机发送回显并提供报文时间戳。[配置文件](canopen_pp/config.py)

## 每轴的五组 PDO

所有 PDO 使用经典 CAN、标准标识符、小端布局和传输类型 1。表中 `n` 是节点号。RPDO3 的第三字节为本阶段经授权加入的模式请求。其余四组沿用项目资料。[原始 PDO 规划](vendor/readme.md)

| 槽 | 通信 / 映射 | CAN-ID | 映射字，依字节顺序 | DLC |
|---|---|---|---|---|
| RPDO1 | 1400 / 1600 | 200h+n | 607A0020、60810020 | 8 |
| RPDO2 | 1401 / 1601 | 300h+n | 60830020、60840020 | 8 |
| RPDO3 | 1402 / 1602 | 400h+n | 60400010、60600008 | 3 |
| TPDO1 | 1800 / 1A00 | 180h+n | 60410010、60630020（Kinco）或 60640020（RobStride）、60610008 | 7 |
| TPDO2 | 1801 / 1A01 | 280h+n | 606C0020 | 4 |

初始化按 ID 选择设备描述。节点 1–3 使用 Kinco EDS。节点 4–7 使用随项目提供的 RS06 EDS，RS00 和 EL05 的映射逐台通过 SDO 验证。初始化读取厂家号及当前状态字，检查 EDS 的对象类型、可映射标记和读写属性。需要重映射时先置 COB-ID 的禁用位，再清零数量、写条目和数量、配置传输类型，最后启用。若额外启用的 RPDO 可改写控制字或模式，初始化禁用该额外槽并读回。所有节点配置结束后再次读回 35 个槽的 COB-ID、传输类型、数量和每个有效条目。SDO Abort 或读回不符会给出节点及对象地址。配置中断时，已变更的映射可能保留在从站 RAM，原因排除后重新执行 `init`。[Kinco EDS](vendor/motor.eds) · [RobStride EDS](vendor/RS06.eds)

进入 Operational 后，程序仅使用同步 PDO 和 SYNC。接收线程记录每次主站 SYNC 回显和两路 TPDO 的时间戳。长度错误使对应反馈失效。使能各阶段需要同一批次 SYNC 或更晚的一组完整反馈，状态通过掩码解析。终端等待输入期间接收及 SYNC 调度继续运行。各 RPDO 按发送回显排序，最后一帧回显后直接发送 SYNC。USB、操作系统调度及 CAN 仲裁会产生实际间隔。在 Kinco 节点 1，前期测试曾观察到类型 1 的控制字在 SYNC 前已影响状态，因此同批报文及后续 TPDO 确认不能证明七个功率级在同一物理时刻动作。[调度器](canopen_pp/communication.py) · [状态机](canopen_pp/controller.py)

## 状态转换与失败处理

对于 Ready、Switched On、Quick Stop Active 或部分已使能的组合，`enable` 先将整组归一到 Switch On Disabled，再逐阶段使能。全部已使能且处于 PP 时重复 `enable` 只确认反馈。Fault、Fault Reaction Active、未知状态、失效反馈会中断使能。中断后输出回退前各轴状态并尝试整组失能。批次传输中断会抑制空闲 SYNC，恢复时必须先以完整的失能批次覆盖待处理控制字。

Kinco 使用 Disable Voltage（`0000h`），RS06、RS00 和 EL05 使用厂家手册 §5.3 的 `0001h`。失能确认目标均为 Switch On Disabled。软件反馈只确认驱动器状态机，涉及承载和安全断电的操作依靠现场机械支撑、抱闸及外部安全回路。[CiA 402](vendor/CanOpen-DS402协议-电机驱动器.pdf) · [Kinco 手册](vendor/iSMD高性能低压伺服一体机使用手册20260303.pdf) · [RS06 手册](vendor/RS06使用说明书260428.pdf) · [RS00 手册](vendor/RS00使用说明书260428.pdf) · [EL05 手册](vendor/EL05使用说明书2600428.pdf)

## 测试和实机记录

```bash
.venv/bin/python -m pytest -q
```

默认测试使用 python-can 虚拟总线和七个 canopen LocalNode SDO 服务，检查正常顺序、映射读回、SDO Abort、部分发送失败、故障及回退、反馈超时、模式和失能确认。实机用例单独开启，会实际使能对应电机并在结束前验证失能。运行时由一个主站控制总线，并确认机构能承受施加和移除转矩。

```bash
CANOPEN_HARDWARE_TEST=rs06 .venv/bin/python -m pytest -q -s tests/test_hardware.py
CANOPEN_HARDWARE_TEST=seven .venv/bin/python -m pytest -q -s tests/test_hardware.py
```

已在 `can0`、1 Mbit/s、100 ms 主站周期先验证 RS06 节点 4，再对七轴做整组验证。整组测试中节点 5–7 初始为 Quick Stop Active。经失能归一后得到以下 TPDO1 实测值。

| 节点 | Shutdown 后 | Switch On 后 | Enable Operation 后 | 使能时 6061 | 最终失能后 |
|---|---|---|---|---|---|
| 1–3，Kinco | 0x4231 | 0x4233 | 0x4637 | 1 | 0x4270 |
| 4–7，RS06 / RS00 / EL05 | 0x0031 | 0x0033 | 0x0037 | 1 | 0x0040 |

独立 SocketCAN 监听器确认每阶段七帧 RPDO3 之后紧接同一帧 SYNC，三次使能阶段观察到的末帧至 SYNC 间隔分别约为 0.52、1.05、1.03 ms。抓包也检查了 NMT Start 后的 SDO 与 RPDO1/2 发送数均为零。七轴第一次初始化曾在 NMT 发送回显等待 300 ms 时超时。初始化 NMT 改用 1 s 回显等待后，实机回归和正式终端流程均通过。故障注入在虚拟总线上完成。[实机回归用例](tests/test_hardware.py)

另以 50 ms 周期执行了一次实机初始化，连续 12 个后续周期均收齐七轴两路 TPDO。独立监听器测得 SYNC 间隔最小 47.803 ms、中位数 50.183 ms、最大 52.227 ms，七轴全程回报 Switch On Disabled。50 ms 下的使能时序及长期抖动尚未实测。
