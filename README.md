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
- 从核异常：Linux 侧发送 `CMD_SAFE_STOP`

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

当前已从 `G431_CAN` 工程中提取 JC4010 电机 CAN 帧格式，并放到：

```text
remote_firmware/jc4010_can.c
remote_firmware/jc4010_can.h
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
sudo ./rpmsg_client /dev/rpmsg0 pvt 1 1000 100 20
sudo ./rpmsg_client /dev/rpmsg0 stop 1
```

`pvt 1 1000 100 20` 含义：

```text
1 号电机
目标位置 10.00 度
速度 100 rpm
力矩百分比 20%
```

从核收到 `CMD_CAN_PVT` 后会打包为 JC4010 的 `0x25` PVT CAN 帧，并通过 `phytium_can_send()` 发送。

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
