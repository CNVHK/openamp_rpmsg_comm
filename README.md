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

## 是否需要从核 ELF 源码

需要。板上通信验证至少要说明两部分代码：

- Linux 端：打开 `/dev/rpmsgX`，发送命令帧。
- 从核端：编译进 `openamp_core0.elf`，在 rpmsg callback 中解析命令并回复。

当前 `remote_firmware/slave_app.c` 给出了从核业务代码框架。真正编译 ELF 时，把其中的 `slave_handle_frame()` 接入飞腾 Standalone OpenAMP 示例工程即可。
