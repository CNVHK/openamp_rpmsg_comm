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

从核固件支持在不重新编译 ELF 的情况下修改俯仰零点和四个 LQR
增益。配置保存在从核 RAM 中，重启从核后恢复固件默认值。所有修改命令
只能在 `disabled` 或 `fault` 状态执行，先停止平衡控制：

```bash
rprun balance-disable
rprun balance-config
```

设置运行时单轮硬超速保护阈值，单位为 `m/s`，允许范围为 `0.5–1.5`。
默认值仍为 `1.0 m/s`，建议按 `1.2`、`1.5` 的顺序逐步测试：

```bash
rprun balance-disable
rprun balance-speed-limit 1.2
rprun balance-config
rprun balance-enable
```

轮径 `0.03225 m` 时，`1.2 m/s` 约为 `355 rpm`，`1.5 m/s` 约为
`444 rpm`。该命令不会修改静止使能条件的低速门槛，也不能关闭超速保护。

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
必须架设保护绳并从小倾角开始验证。恢复编译默认配置：

```bash
rprun balance-reset-config
```

当俯仰角绝对值达到 `3` 度，并且位置/速度力矩与姿态恢复力矩方向相反时，
控制器会临时屏蔽相反的行走力矩，优先使用电机力矩恢复姿态。回到阈值以内后
自动恢复完整四状态 LQR。

`pvt 1 1000 100 20` 含义：

```text
1 号电机
目标位置 10.00 度
速度 100 rpm
力矩百分比 20%
```

从核收到 `CMD_CAN_PVT` 后会打包为 `0x25` PVT CAN 帧，并通过 `phytium_can_send()` 发送。`test` 命令会初始化两台电机，并让两台电机各转动一圈。

## 云台电机测试

`linux_user/gimbal_test.c` 是主核 Linux 侧的云台测试程序，固定使用：

- CAN ID 3：yaw 电机
- CAN ID 4：pitch 电机

在飞腾派 Linux 上编译：

```bash
make gimbal
```

`torque-test` 仅用于排查 ID 1/2 轮电机的力矩模式。力矩范围为 -0.22–0.22 N·m，脉冲时间为 20–2000 ms；命令结束后会自动发送零力矩并进入 idle。长时间测试前必须先用机械方式约束车轮，不能用手接触旋转中的车轮。

读取电机驱动器错误寄存器 `0x000C`：

```bash
sudo ./build/rpmsg_client /dev/rpmsg0 motor-fault 1
sudo ./build/rpmsg_client /dev/rpmsg0 motor-fault 2
```

轮速三方对照诊断只能在平衡控制停用且车轮架空或机械约束时运行。它会短时施加不超过
`0.10 N*m` 的力矩，同时比较周期反馈 `0x2A`、实时速度寄存器 `0x0006`
和编码器位置差分得到的速度，结束后自动清零力矩并进入 idle：

```bash
rprun balance-disable
sudo ./build/rpmsg_client /dev/rpmsg0 motor-speed-diag 1 0.05 1000
sudo ./build/rpmsg_client /dev/rpmsg0 motor-speed-diag 2 -0.05 1000
```

当前平衡控制根据实机日志对周期反馈速度应用 `0.5` 标定系数；原始 rpm
仍由 `balance-status` 输出。俯仰角速度在进入 LQR 前经过 10 Hz 一阶低通，
默认 `K2=-0.4`。`balance-config` 会显示这两个固定参数。

首次安装时，先卸载负载或架空云台，并手动把 yaw、pitch 放到机械中位。确认位置无误后执行永久标零：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 setzero CONFIRM
```

该命令分别向 ID 3 和 ID 4 写入永久原点寄存器 `0x00A6`。驱动器会保存当前位置偏置并重启；通常只需要执行一次。重新安装电机、驱动板或机械结构发生变化时才重新标零。

驱动器重启完成后执行日常初始化。初始化只设置位置梯形轨迹模式并进入闭环，不会修改已经保存的原点：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 init
```

先用低速、小角度测试。`set` 的角度单位是度，后两个可选参数分别是转速 rpm 和力矩百分比：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 set 5 0
sudo ./build/gimbal_test /dev/rpmsg0 set 0 5 20 10
sudo ./build/gimbal_test /dev/rpmsg0 center
```

默认扫动幅度为正负 10 度、20 rpm、10% 力矩，只执行一轮：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 sweep
sudo ./build/gimbal_test /dev/rpmsg0 sweep 5 2
```

持续小角度循环测试默认使用正负 5 度，并一直运行到按下 `Ctrl+C`。也可以指定不超过 10 度的幅度：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 test
sudo ./build/gimbal_test /dev/rpmsg0 test 3
```

`test` 不会初始化或修改原点，运行前必须先执行一次 `init`。

停止 ID 3 和 ID 4，或读取通信状态：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 stop
sudo ./build/gimbal_test /dev/rpmsg0 status
```

扫动过程中按 `Ctrl+C` 会向两台电机发送停止命令。程序的软件角度限制当前为 yaw 正负 180 度、pitch 正负 90 度；正式带机构测试前，应按实际机械限位修改 `linux_user/gimbal_test.c` 中的限制值。两个电机命令通过 RPMsg 依次发送，因此该程序适合功能测试，不是严格同步的实时云台控制器。

通用客户端的 `zero <id>` 使用 `0x00A7`，只是临时原点，驱动器重启后失效；`setorigin <id>` 使用 `0x00A6`，会永久保存原点并重启驱动器。不要把两者混用。

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
