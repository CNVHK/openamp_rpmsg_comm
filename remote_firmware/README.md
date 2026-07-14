# 从核裸机固件代码说明

`slave_app.c` 是从核裸机端业务代码框架，用于集成到飞腾 Standalone OpenAMP 示例工程中，最终编译成：

```text
openamp_core0.elf
```

建议集成位置：

```text
phytium-pi-os/output/build/phytium-standalone-openamp-v1.0/example/system/amp/openamp_for_linux/src/slaver_00_example.c
```

当前实现内容：

- 解析 Linux 主核发送的命令帧。
- 支持心跳、CAN 电机、4 路 PWM 舵机和 BMI088 命令。
- 支持 `CMD_MOTOR_TEST` 初始化两台电机并各转动一圈。
- 校验失败或未知命令时丢弃消息。
- 返回 CAN、舵机和 BMI088 的综合诊断状态。

说明：

由于飞腾 Standalone SDK 的 OpenAMP 初始化、resource table、IPI 和 rpmsg endpoint 创建代码由官方示例提供，本文件只实现业务命令和硬件控制逻辑。
