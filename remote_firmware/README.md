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
- 支持 `CMD_HEARTBEAT`、`CMD_SET_MOTOR`、`CMD_SAFE_STOP`。
- 校验失败时进入安全状态。
- 预留 `apply_motor_output()`，后续接入 PWM/GPIO/电机驱动。
- 给出 rpmsg callback 集成示例。

说明：

由于飞腾 Standalone SDK 的 OpenAMP 初始化、resource table、IPI 和 rpmsg endpoint 创建代码由官方示例提供，本文件只写业务层和安全逻辑。初赛阶段这样更容易说明，也方便后续迁移。
