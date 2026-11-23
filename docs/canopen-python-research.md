# Python CANopen 伺服 PP 源码调研

> 历史调研，记录原七轴编号下的早期 PP 运动原型。当前八轴同步 PDO 终端的协议、命令和实机记录见[项目说明](../README.md)。下文的 SDO 使能和位置运动流程属于上一阶段。

最值得对照的三份代码是本项目已经使用的 canopen-python、带有 maxon 和 FAULHABER 驱动适配的 Being，以及 Analog Devices 的 PyTrinamic 单轴 PP 示例。以下源码链接固定到实际阅读的版本。三个项目分别展示了协议接口、伺服应用和 PDO 触发，本次检索未找到覆盖这套 Kinco iSMD、RS06、RS00 和 EL05 七节点组合的完整 Python 实现。

## 对七轴程序的结论

| 环节 | 本项目应采用的验收依据 |
|---|---|
| `enable` | 按现场实测，七轴逐轴写入并读回 6060h=1，确认 000Fh 使能控制字的 SDO 应答，随后等待主站下一帧 SYNC 的 TX 回显，再同时核对 6041h 为 Operation Enabled、6061h 为 1。七轴的新 TPDO1 都确认后，才允许 `start`。[当前使能实现](../canopen_pp/controller.py#L82-L153) |
| `start` 与重复启动 | 先下发七轴共 14 个参数 RPDO，经过 SYNC 后逐轴读回目标、速度和两条斜率。随后让七个含 Bit 4 上升沿的控制字位于同一对相邻 SYNC 之间。以每轴触发 SYNC 后新 TPDO 的 Bit 12 置位确认接受，清除 Bit 4 并等待 Bit 12 复位。到位再看 Bit 10 及实测位置窗口。[本项目批次逻辑](../canopen_pp/controller.py#L217-L282) · [批次发送和 SYNC](../canopen_pp/communication.py#L148-L175) · [CiA DSP 402 §12.4 图 17](../vendor/CanOpen-DS402协议-电机驱动器.pdf) |
| `stop` 和 `quickstop` | 在 PP 下，Halt 时用状态字及 Bit 10 确认驱动器报告完成减速。Quick Stop 需结合实际选项和返回状态解释，进入 Quick Stop Active 时可能仍在减速。RS06、RS00、EL05 的实际制动斜率、停止距离和恢复行为仍须在对应设备上验证。[本项目停止确认](../canopen_pp/controller.py#L300-L360) · [Kinco 手册](../vendor/iSMD高性能低压伺服一体机使用手册20260303.pdf) · [现场验收项](../README.md#现场运动验收仍需完成) |

若要用一份实机记录核对开源示例与现场的差异，最有区分力的是保留同一时间轴上的 SYNC、七个 RPDO3、七个 TPDO1 和参数 SDO 读回。两次连续执行相同绝对目标也应各有一次独立的 Bit 12 高、低应答，即使第二次几乎没有位移。本次研究只阅读源码并验证了本地 Python 库行为，设备模式显示时序依据用户实测，未执行实机使能、运动或制动试验。[项目总线布局](../vendor/readme.md) · [现有虚拟总线测试](../tests/test_virtual.py)

## Python 开源实现

### canopen-python/canopen v2.4.1（MIT）

这是本项目已安装的版本。先读 `BaseNode402.check_statusword()` 和 `op_mode`。周期 TPDO 映射了状态字时，前者等待一帧新反馈。对象未映射时可以走 SDO。模式 setter 则在写 6060h 后立即等待 6061h，路径内没有“发使能控制字，再等待下一帧 SYNC”这一步。该版本会把部分 SDO 通信错误和运行超时记为 warning，赋值调用返回本身不足以让应用宣布 PP 已就绪。[固定版本](https://github.com/canopen-python/canopen/releases/tag/v2.4.1) · [模式设置与确认](https://github.com/canopen-python/canopen/blob/317fd3d61513b33ddb1aaf8a3ea2c23384a275ee/canopen/profiles/p402.py#L395-L454) · [状态反馈与控制字](https://github.com/canopen-python/canopen/blob/317fd3d61513b33ddb1aaf8a3ea2c23384a275ee/canopen/profiles/p402.py#L465-L519) · [许可证](https://github.com/canopen-python/canopen/blob/317fd3d61513b33ddb1aaf8a3ea2c23384a275ee/LICENSE.txt)

**离线验证。** 使用本项目的 `canopen==2.4.1`、`RS06.eds` 和节点号 5，将 6040h 映射到传输类型 1 的 RPDO3。用内存记录器接收库的发送调用，没有打开 CAN 接口，也没有启动后台 PDO 发送任务。结果如下。[同步 PDO 判定](https://github.com/canopen-python/canopen/blob/317fd3d61513b33ddb1aaf8a3ea2c23384a275ee/canopen/pdo/base.py#L294-L310) · [单次 PDO 发送](https://github.com/canopen-python/canopen/blob/317fd3d61513b33ddb1aaf8a3ea2c23384a275ee/canopen/pdo/base.py#L523-L525)

| 调用 | 本地控制字 | 新增发送调用 |
|---|---:|---|
| `node.controlword = 0x000F` | 15 | 0 次 |
| `rpdo3.transmit()` | 15 | 1 次，CAN-ID `0x405`，数据 `0F 00` |

此外，库的模式 setter 先通过 6502h 查询支持的模式。用库自带的 EDS 解析器检查，本项目的 `RS06.eds` 声明了 6502h，Kinco 的 `motor.eds` 没有这个条目。若直接用同一 setter 接管七轴，就会增加当前 Kinco 对象描述无法满足的依赖。[模式支持查询](https://github.com/canopen-python/canopen/blob/317fd3d61513b33ddb1aaf8a3ea2c23384a275ee/canopen/profiles/p402.py#L441-L454) · [Kinco EDS](../vendor/motor.eds) · [RobStride EDS](../vendor/RS06.eds)

### rauc-lab/being（MIT）

这是实际用于 PATHOS 机器人艺术项目的 Python 应用。源码包含 FAULHABER MCLM 3002 和 maxon EPOS4 的独立适配，以及状态切换测试。核对的提交日期为 2022-08-29，适合研究应用结构和设备差异。[应用背景](https://github.com/rauc-lab/being/blob/98d4ca5f0ca10c953ec742a4bb3f11f0423e425b/README.rst) · [设备适配](https://github.com/rauc-lab/being/blob/98d4ca5f0ca10c953ec742a4bb3f11f0423e425b/being/motors/controllers.py#L405-L487) · [状态机测试](https://github.com/rauc-lab/being/blob/98d4ca5f0ca10c953ec742a4bb3f11f0423e425b/tests/test_can.py#L86-L140) · [许可证](https://github.com/rauc-lab/being/blob/98d4ca5f0ca10c953ec742a4bb3f11f0423e425b/LICENSE)

`CiA402Node.move_to()` 是一段容易读懂的单轴 PP 代码。它先清控制字 Bit 4，通过 SDO 写目标和可选的速度、斜率，再置 Bit 4。默认还置 Bit 5，允许立即变更目标。该方法把同一斜率写给加速和减速，函数内部没有等待 Bit 12 应答。主循环另有批量发送 RPDO 的机制，但 `move_to()` 的目标和触发走的是单轴 SDO。逐次调用七个 `move_to()` 不会得到同一 SYNC 的七轴触发。[PP 方法](https://github.com/rauc-lab/being/blob/98d4ca5f0ca10c953ec742a4bb3f11f0423e425b/being/can/cia_402.py#L349-L376) · [周期循环](https://github.com/rauc-lab/being/blob/98d4ca5f0ca10c953ec742a4bb3f11f0423e425b/being/being.py#L136-L150)

Being 记录过多个 RPDO 同时映射 6040h 的情况，后一帧可能把前一帧的控制字覆盖。本项目的探测和初始化现已检查额外启用的 RPDO 映射。即使 CAN-ID 各不相同，也会报告哪些映射能改写同一运动对象。控制期间收到该 CAN-ID 的数据帧时，程序报告实际冲突。[Being 的映射说明](https://github.com/rauc-lab/being/blob/98d4ca5f0ca10c953ec742a4bb3f11f0423e425b/being/can/cia_402.py#L65-L99) · [本项目映射检查](../canopen_pp/pdo.py#L36-L66)

### Analog Devices PyTrinamic · 单轴 CANopen PP 的 PDO 实例

TMCM-1230 示例用 Python 的 `canopen` 接口，在 RPDO4 同帧映射 `6040h` 控制字与 `607Ah` 目标位置，在 TPDO4 同帧采集 `6064h` 实际位置与 `606Ch` 实际速度，按接收时间记录轨迹。它以 `000Fh` 为基线，对每个目标发 `001Fh` 再发 `000Fh`，随后通过 SDO 等待 `6041h` Bit 10 到位。可参考映射顺序、Bit 4 边沿和反馈采集。示例没有等待 Bit 12 应答，也没有发送 SYNC。两个 PDO 还使用同一 COB-ID `0x381`。七轴项目应沿用自己的报文标识、ACK 超时和同步边界。[PDO 映射与反馈采集](https://github.com/analogdevicesinc/PyTrinamic/blob/0ff4296c88d185fd1ecd448169df7d65ad5c3b6c/examples/modules/TMCM123x_0_1/CANopen/six_point_ramp_demo_using_pdo.py#L23-L64) · [触发和到位循环](https://github.com/analogdevicesinc/PyTrinamic/blob/0ff4296c88d185fd1ecd448169df7d65ad5c3b6c/examples/modules/TMCM123x_0_1/CANopen/six_point_ramp_demo_using_pdo.py#L83-L106)

`TmcmNode` 根据状态字掩码选择 `0006h → 0007h → 000Fh` 中尚需执行的转换。PP 示例先使能、后选模式，无法验证本机要求的使能顺序。仓库另有 TMCM-6214 的双轴示例，作者标明测试固件为 3.22，实际是同一节点的两根轴逐次用 SDO 写轮廓速度。仓库使用 MIT 许可证，有正式发布 v0.2.26。[状态机](https://github.com/analogdevicesinc/PyTrinamic/blob/0ff4296c88d185fd1ecd448169df7d65ad5c3b6c/pytrinamic/modules/canopen_node.py#L35-L84) · [双轴示例](https://github.com/analogdevicesinc/PyTrinamic/blob/0ff4296c88d185fd1ecd448169df7d65ad5c3b6c/examples/modules/TMCM6214/CANopen/rotate_demo_using_sdo_2-axis.py#L8-L50) · [许可证](https://github.com/analogdevicesinc/PyTrinamic/blob/0ff4296c88d185fd1ecd448169df7d65ad5c3b6c/LICENSE) · [发布记录](https://github.com/analogdevicesinc/PyTrinamic/releases/tag/v0.2.26)

## 一个与现场时序接近的公开示例

`freezeLUO/ZeroErr_Motor_CanOpen` 的旧版多电机示例中，`enable_motor()` 每次发送 0006h、0007h、000Fh 之一，随后显式发送 SYNC。值得比较的是报文顺序。该方法每步固定等待 100 ms，最后打印使能完成，没有在此处读取 6041h 或 6061h。另一个函数用 Bit 10 检查目标到达。这是针对 ZeroErr 的公开示例，此处只引用链接。[使能及到位检查](https://github.com/freezeLUO/ZeroErr_Motor_CanOpen/blob/4c6829d0e9fdbc70aebc953e607b65f3e50e7b95/old/MultiMotorControl_PP.py#L284-L318) · [触发实现](https://github.com/freezeLUO/ZeroErr_Motor_CanOpen/blob/4c6829d0e9fdbc70aebc953e607b65f3e50e7b95/old/MultiMotorControl_PP.py#L331-L347)

## Novanta IngeniaMotion / IngeniaLink · 厂商伺服 SDK 的反馈与超时

两库提供了不同层次的参考。IngeniaLink 的 Linux 示例用 SocketCAN、Node ID 和厂商字典连接驱动器，底层把字典解析出的索引、子索引传给 SDO。IngeniaMotion 的定位示例走 EtherCAT，设置轮廓速度、加减速度，逐个提交绝对位置并设 2 秒等待上限。其运动函数把写目标、产生锁存边沿、按实际位置误差等待到位拆开，可参考这一接口设计。[Linux CANopen 连接](https://github.com/ingeniamc/ingenialink-python/blob/8804f77ffb843c9ea06f1e320aa4701cd83ef77b/examples/canopen/can_connection_linux.py#L15-L26) · [SDO 实现](https://github.com/ingeniamc/ingenialink-python/blob/8804f77ffb843c9ea06f1e320aa4701cd83ef77b/ingenialink/canopen/servo.py#L103-L127) · [定位示例](https://github.com/ingeniamc/ingeniamotion/blob/f8d7031ff152714de9e21f691573e95bd435e34f/examples/position_ramp.py#L6-L29) · [位置等待](https://github.com/ingeniamc/ingeniamotion/blob/f8d7031ff152714de9e21f691573e95bd435e34f/ingeniamotion/motion.py#L705-L726)

该 SDK 的 `PROFILE_POSITION` 枚举值为 `0x14`，锁存使用 `DRV_STATE_CONTROL` 的 `0x0200`，等待条件是读回控制寄存器的同一位。这与本机的 `6060h=1`、Bit 4 触发、状态字 Bit 12 确认各有明确区别。工程成熟度可从模拟边沿失败的单测、0.10.5 变更记录核对。两个仓库均标注 CC BY-NC-ND 4.0，本次仅比较设计。[模式枚举](https://github.com/ingeniamc/ingeniamotion/blob/f8d7031ff152714de9e21f691573e95bd435e34f/ingeniamotion/enums/__init__.py#L49-L64) · [锁存实现](https://github.com/ingeniamc/ingeniamotion/blob/f8d7031ff152714de9e21f691573e95bd435e34f/ingeniamotion/motion.py#L21-L106) · [边沿测试](https://github.com/ingeniamc/ingeniamotion/blob/f8d7031ff152714de9e21f691573e95bd435e34f/tests/test_motion.py#L92-L155) · [变更记录](https://github.com/ingeniamc/ingeniamotion/blob/f8d7031ff152714de9e21f691573e95bd435e34f/CHANGELOG.md#L1-L24) · [IngeniaMotion 许可证](https://github.com/ingeniamc/ingeniamotion/blob/f8d7031ff152714de9e21f691573e95bd435e34f/LICENSE.md) · [IngeniaLink 许可证](https://github.com/ingeniamc/ingenialink-python/blob/8804f77ffb843c9ea06f1e320aa4701cd83ef77b/LICENSE.md)

## RobStride 与 Kinco · 匹配型号时还需匹配总线协议

RobStride 官方 MIT 许可的 Python 示例演示了失能、选模式 1、使能、连续定位，但参数实际通过厂商 29 位扩展帧传输，速度、加速度、目标依次写私有索引 `7024h`、`7025h`、`7016h`。它能解释同款电机为何也有名为 PP 的 Python 示例，不能作为 CANopen `6040h/6041h` 或 SYNC 的证据。[官方示例](https://github.com/RobStride/Python_Sample/blob/cbf977e56c842d57a65f3f17c1b1ecaef002c424/examples/position_control.py#L89-L115) · [帧编码](https://github.com/RobStride/Python_Sample/blob/cbf977e56c842d57a65f3f17c1b1ecaef002c424/robstride_dynamics/bus.py#L226-L242) · [PP 写操作](https://github.com/RobStride/Python_Sample/blob/cbf977e56c842d57a65f3f17c1b1ecaef002c424/robstride_dynamics/bus.py#L517-L551) · [索引](https://github.com/RobStride/Python_Sample/blob/cbf977e56c842d57a65f3f17c1b1ecaef002c424/robstride_dynamics/protocol.py#L66-L90) · [许可证](https://github.com/RobStride/Python_Sample/blob/cbf977e56c842d57a65f3f17c1b1ecaef002c424/LICENSE)

第三方 motorbridge 的实验性 `robstride_cia402` 路径更接近本机。它列出 RS00 和 RS06，Rust 实现通过 SDO 在失能时设 PP，配置力矩、速度、加速度，再使能并直接写 `607Ah`。该函数没有七节点 SYNC 或 Bit 12 握手，型号表也未列 EL05。它与随仓三款手册 §5.6 提供了同一个待核实现象：驱动器可能在写目标时即启动。现有公开样例不能证明这套固件可以提前装载不同目标并等待统一触发。本次也未找到可核验的 Kinco iSMD 同型号厂商 Python CANopen PP 实例。其 Bit 4/Bit 12 与单位换算以随仓手册为依据。[型号表](https://github.com/motorbridge/motorbridge/blob/7aabc9851114646a7b2a3c28599e90e6d9802de1/motor_vendors/robstride_cia402/src/motor.rs#L10-L65) · [使能](https://github.com/motorbridge/motorbridge/blob/7aabc9851114646a7b2a3c28599e90e6d9802de1/motor_vendors/robstride_cia402/src/motor.rs#L341-L379) · [PP 实现](https://github.com/motorbridge/motorbridge/blob/7aabc9851114646a7b2a3c28599e90e6d9802de1/motor_vendors/robstride_cia402/src/motor.rs#L426-L466) · [实验范围](https://github.com/motorbridge/motorbridge/blob/7aabc9851114646a7b2a3c28599e90e6d9802de1/README.md#L275-L284) · [RS06 手册](../vendor/RS06使用说明书260428.pdf) · [RS00 手册](../vendor/RS00使用说明书260428.pdf) · [EL05 手册](../vendor/EL05使用说明书2600428.pdf) · [Kinco iSMD 手册](../vendor/iSMD高性能低压伺服一体机使用手册20260303.pdf)
