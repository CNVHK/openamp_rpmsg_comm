# OpenAMP 核间通信

## 功能目标

本项目对应“OpenAMP 核间通信”。基础版完成一个简单可靠的命令帧格式，用于 Linux 主核和从核之间传输结构化数据。

命令帧包含：

- 帧头
- 类型
- 序号
- 长度
- 负载
- 校验

异常处理设计：

- 校验错误：丢弃帧
- 长度错误：丢弃帧
- 超时：Linux 侧重发或进入安全状态
- 从核异常：Linux 侧发送 `CMD_CAN_SAFE_STOP`

## 直接验证

```bash
make test
./build/test_protocol
```

完整部署和板上验证步骤见：

```text
DEPLOY_VERIFY.md
```

## 板上验证步骤

1. 飞腾派启动 Linux。
2. 启动从核：

```bash
echo start | sudo tee /sys/class/remoteproc/remoteproc0/state
```

3. 检查通道：

```bash
ls /sys/bus/rpmsg/devices/
ls /dev/rpmsg*
```

4. 绑定字符设备驱动：

```bash
cd /sys/bus/rpmsg/devices/virtio0.rpmsg-openamp-demo-channel.-1.0
echo rpmsg_chrdev | sudo tee driver_override
sudo modprobe rpmsg_char
```

5. Linux 端可参考 `linux_user/rpmsg_client.c`，从核裸机端可参考 `remote_firmware/slave_app.c`。

## CAN 电机控制接入

电机 CAN 帧组装代码位于：

```text
remote_firmware/motor_can.c
remote_firmware/motor_can.h
```

飞腾从核 CAN 适配层在：

```text
remote_firmware/phytium_can_port.c
remote_firmware/phytium_can_port.h
```

Linux 客户端支持以下命令：

```bash
sudo ./rpmsg_client /dev/rpmsg0 heartbeat
sudo ./rpmsg_client /dev/rpmsg0 enable 1
sudo ./rpmsg_client /dev/rpmsg0 zero 1
sudo ./rpmsg_client /dev/rpmsg0 test
sudo ./rpmsg_client /dev/rpmsg0 torque-test 1 0.05 200
sudo ./rpmsg_client /dev/rpmsg0 pvt 1 1000 100 20
sudo ./rpmsg_client /dev/rpmsg0 stop 1
```

## 平衡控制运行时配置

从核固件支持在不重新编译 ELF 的情况下修改俯仰零点、四个 LQR 增益、
角速度滤波、姿态优先角、轮速保护和单轮力矩限制。配置保存在从核 RAM 中，
重启从核后恢复固件默认值。所有修改命令
只能在 `disabled` 或 `fault` 状态执行，先停止平衡控制：

```bash
rprun balance-disable
rprun balance-config
```

设置运行时单轮硬超速保护阈值，单位为 `m/s`，允许范围为 `0.2–2.0`。
默认值仍为 `1.0 m/s`：

```bash
rprun balance-disable
rprun balance-speed-limit 1.2
rprun balance-config
rprun balance-enable
```

轮径 `0.03225 m` 时，`1.2 m/s` 约为 `355 rpm`。该命令不会修改静止
使能条件的低速门槛，也不能关闭超速保护。超过默认值只用于有保护绳的诊断，
不能用来掩盖发散或振荡。

`balance-trim` 设置机器人处于真实机械直立位置时 IMU 应扣除的俯仰角，
单位为度，允许范围为 `-5` 到 `+5` 度。例如机械直立时状态显示
`pitch=+1.0 deg`：

```bash
rprun balance-trim 1.0
rprun balance-config
```

设置四个直接总力矩 LQR 增益：

```bash
rprun balance-gains -3.759674 -0.486785 -0.062457 -0.247058
rprun balance-config
```

增益顺序固定为 `K_theta K_theta_rate K_position K_velocity`。固件只接受
与当前模型符号一致的有限范围参数，但范围检查不能证明参数一定稳定；新参数
必须架设保护绳并从小倾角开始验证。

设置俯仰角速度一阶低通截止频率，允许 `5–40 Hz`，默认 `20 Hz`：

```bash
rprun balance-disable
rprun balance-filter 20
rprun balance-config
```

设置姿态优先角，允许 `1–10` 度，默认 `3` 度：

```bash
rprun balance-posture-angle 3
```

设置单轮力矩限制，允许 `0.05–0.30 N*m`，默认 `0.22 N*m`：

```bash
rprun balance-torque-limit 0.22
```

`0.22 N*m` 以上只用于确认电机、驱动器、电池和机械结构均允许更高输出后的
短时保护绳测试。提高该限制会增加电机电流、跌倒冲击和机械损坏风险。

恢复编译默认配置：

```bash
rprun balance-reset-config
```

当俯仰角绝对值达到 `3` 度，并且位置/速度力矩与姿态恢复力矩方向相反时，
控制器会临时屏蔽相反的行走力矩，优先使用电机力矩恢复姿态。回到阈值以内后
自动恢复完整四状态 LQR。

## 高频平衡日志

`balance_logger` 使用一个常驻进程和轻量 RPMsg 遥测回复，不再为每个样本
启动一次 `sudo rpmsg_client`。默认采样率为 20 Hz，最高允许 50 Hz。推荐由
日志器负责启用平衡控制；按 `Ctrl+C`、收到 `SIGTERM` 或达到指定时长时，
它会自动发送 `balance-disable`：

```bash
make logger
sudo ./build/balance_logger --rate 20 --enable --stop-on-fault
```

短时间 50 Hz 测试并在 30 秒后自动停机：

```bash
sudo ./build/balance_logger --rate 50 --duration 30 --enable --stop-on-fault
```

只记录已经运行的控制器，不负责启停：

```bash
sudo ./build/balance_logger --rate 20
```

日志默认写入 `logs/balance/`，每小时创建新 CSV，并在切换后后台压缩上一小时
文件。即使使用 `sudo`，程序也会在打开 RPMsg 设备后恢复为原用户身份，因此
日志不会变成 root 所有。CSV 包含 `read_ok`、RPMsg `latency_us`、从核
`loop_count`、真实 `state/fault`、LQR 状态、力矩、电流和原始轮速。

同一个 `/dev/rpmsg0` 上的多个读取者可能竞争回复。日志器运行期间不要另开
终端执行 `rprun`；它已经每秒显示一次状态、故障、俯仰角、速度、实际采样率
和累计错误数。需要修改配置时，先按 `Ctrl+C` 停止日志器。

`pvt 1 1000 100 20` 含义：

```text
1 号电机
目标位置 10.00 度
速度 100 rpm
力矩百分比 20%
```

从核收到 `CMD_CAN_PVT` 后会打包为 `0x25` PVT CAN 帧，并通过 `phytium_can_send()` 发送。`test` 命令会初始化两台电机，并让两台电机各转动一圈。

`torque-test` 仅用于排查 ID 1/2 轮电机的力矩模式。力矩范围为 -0.22–0.22 N·m，脉冲时间为 20–2000 ms；命令结束后会自动发送零力矩并进入 idle。长时间测试前必须先用机械方式约束车轮，不能用手接触旋转中的车轮。

读取电机驱动器错误寄存器 `0x000C`：

```bash
sudo ./build/rpmsg_client /dev/rpmsg0 motor-fault 1
sudo ./build/rpmsg_client /dev/rpmsg0 motor-fault 2
```

轮速三方对照诊断只能在平衡控制停用且车轮架空或机械约束时运行。它会短时施加不超过 `0.10 N*m` 的力矩，同时比较周期反馈 `0x2A`、实时速度寄存器 `0x0006` 和编码器位置差分得到的速度，结束后自动清零力矩并进入 idle：

```bash
rprun balance-disable
sudo ./build/rpmsg_client /dev/rpmsg0 motor-speed-diag 1 0.05 1000
sudo ./build/rpmsg_client /dev/rpmsg0 motor-speed-diag 2 -0.05 1000
```

实机三方对照确认周期反馈速度、`0x0006` 寄存器速度和位置差分速度误差小于约 1.2%，因此平衡控制使用 `1.0` 速度系数。

## 云台控制和安全标定

`linux_user/gimbal_test.c` 是 Linux 侧云台控制和标定工具，固定使用 CAN ID 3 作为 yaw、ID 4 作为 pitch。编译命令为：

```bash
make gimbal
```

### 永久零点

首次安装时先可靠托住云台，将 yaw、pitch 手动放到机械中位。确认摄像头和支架都有足够活动空间后执行：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 setzero CONFIRM
```

该命令向两台驱动器写入永久原点寄存器 `0x00A6`。驱动器会保存当前位置并重启；只有重新安装电机、驱动板或机械结构变化时才需要再次执行。通用客户端的 `zero <id>` 使用临时原点 `0x00A7`，不能替代永久零点。

### 软件安全范围

永久零点完成后必须标定四个软件边界。边界应留在真正碰撞位置以内，不能把机械止挡或线缆刚好拉紧的位置作为软件边界。

保持从核状态为 `disabled`，手动移动对应轴到安全位置后分别确认：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 limit yaw min CONFIRM
sudo ./build/gimbal_test /dev/rpmsg0 limit yaw max CONFIRM
sudo ./build/gimbal_test /dev/rpmsg0 limit pitch min CONFIRM
sudo ./build/gimbal_test /dev/rpmsg0 limit pitch max CONFIRM
sudo ./build/gimbal_test /dev/rpmsg0 status
```

只有显示 `limits: valid=0x0f` 才允许启动。最小值必须为负角度、最大值必须为正角度，永久零点必须位于每一对边界之间。

软件边界目前保存在从核 RAM 中，从核重启后会清除。记录好四个实机角度后，可以直接恢复：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 limits -60 60 -25 35 CONFIRM
```

上面的数字只是格式示例，必须替换成实机标定结果。清除全部边界使用 `reset-limits CONFIRM`。

### 启动归零和位置控制

边界有效后启动。两台电机进入位置模式，并以 `5 rpm` 缓慢归零；状态依次经过 `starting`、`homing`、`active`：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 enable
sudo ./build/gimbal_test /dev/rpmsg0 status
```

`init` 是 `enable` 的兼容别名。只有状态为 `active` 才接受目标。yaw 和 pitch 通过一条原子 RPMsg 命令提交，从核先同时检查两轴边界，再连续发出两条 CAN 帧：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 set 5 0
sudo ./build/gimbal_test /dev/rpmsg0 set 0 5 20 10
sudo ./build/gimbal_test /dev/rpmsg0 center
sudo ./build/gimbal_test /dev/rpmsg0 sweep 5 2
```

持续循环测试使用 `test [angle_deg]`，按 `Ctrl+C` 会请求受控关闭。测试不会自动启动、标零或恢复边界。

归零超过 12 秒、任一反馈超过 150 ms 未更新、实际位置越过边界 1 度、目标未在按角差和速度计算的期限内到位或 CAN 发送失败时，从核会立即让两台电机进入 idle。故障位为：`0x01` yaw 反馈、`0x02` pitch 反馈、`0x04` CAN、`0x08` 边界配置、`0x10` 实际越界、`0x20` 归零超时、`0x40` 流式命令超时、`0x80` 运动到位超时/疑似卡滞。

### 受控关闭和紧急停止

正常关闭先保持当前实际位置，每 100 ms 降低 2% 力矩，约 0.5 秒后进入 idle。应等待状态变成 `disabled` 再断电：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 disable
sudo ./build/gimbal_test /dev/rpmsg0 status
```

`stop` 是 `disable` 的兼容别名。即将碰撞、机构卡死或其他紧急情况使用立即 idle，不经过力矩缓降：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 estop
```

`status` 会显示目标和实际角度、速度、电流、反馈年龄、软件边界、运行状态和故障。正常缓降只能减少突然失力，无法保证所有重心和摩擦条件下摄像头都不下坠，机械结构仍应配置限位和必要的阻尼或防坠措施。

## 从核工程必须配置

在飞腾 Pi OS 从核工程配置文件中至少启用：

```text
CONFIG_USE_FCAN=y
```

配置文件通常在：

```text
phytium-pi-os/output/build/phytium-standalone-openamp-v1.0/example/system/amp/openamp_for_linux/configs/phytiumpi_aarch64_firefly_openamp_core0.config
```

如果编译提示 `fio_mux.h`、`fcan.h` 等头文件找不到，需要参考：

```text
example/peripherals/can/can/configs/
```

把 CAN 示例中的 IOMUX/CAN 相关配置同步到从核配置文件。

还需要在 `slaver_00_example.c` 初始化阶段调用：

```c
#include "phytium_can_port.h"

phytium_can_init();
```

## 是否需要从核 ELF 源码

需要。板上通信验证至少要说明两部分代码：

- Linux 端：打开 `/dev/rpmsgX`，发送命令帧。
- 从核端：编译进 `openamp_core0.elf`，在 rpmsg callback 中解析命令并回复。

当前 `remote_firmware/slave_app.c` 给出了从核业务代码框架。真正编译 ELF 时，把其中的 `slave_handle_frame()` 接入飞腾 Standalone OpenAMP 示例工程即可。
