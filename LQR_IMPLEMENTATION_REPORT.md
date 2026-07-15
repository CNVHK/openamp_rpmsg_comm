# OpenAMP 从核 LQR 基础控制实施报告

日期：2026-07-15

## 目标与范围

第一版控制对象为轮腿机器人左右轮电机，腿高在启用前由机械结构或已有执行机构固定，LQR 运行期间不改变腿高。状态量为机身俯仰角、俯仰角速度、平均轮位移和平均轮速度；输出为左右轮电机力矩。

本次没有加入转向、行驶速度指令、腿高调节、位置跟踪积分器或 Linux 侧实时控制环。

## 已实现内容

- 新增独立 `lqr_controller`，实现启用瞬间轮位置归零、状态反馈、总水平力到单轮力矩换算、力矩限幅、倾倒保护和非法浮点数保护。
- 使用增益 `K = [-119.5795, -14.0267, -1.4142, -4.8475]`。
- 新增 CAN 力矩指令，写寄存器 `0x0020`，单位按协议定义为 `0.01 N·m`。
- 新增 CAN 反馈解析，解析 `0x580 + motor_id`、命令字 `0x2A` 的位置、转速和电流反馈。
- 新增 CAN RX FIFO 非阻塞轮询、每个电机的最新反馈缓存和通用定时器时间戳。
- BMI088 数据转换为 `m/s²` 和 `rad/s`，启用时采集 100 个静止样本校准陀螺零偏，并用互补滤波生成 pitch。
- 新增 100 Hz 从核控制状态机：`disabled -> arming -> active`，故障进入 `fault` 并发送零力矩和 idle 命令。
- OpenAMP 主循环改为 `platform_poll_nonblocking()`，RPMsg 通信不再阻塞控制周期。
- 新增 RPMsg 命令 `balance-enable`、`balance-disable` 和 `balance-status`。
- ACK 扩展到 120 字节，返回状态、故障位、控制频率、姿态、轮状态、左右力矩和循环次数。
- 主核 `rpmsg_client` 已支持构建和解码上述命令。

## 默认参数

| 参数 | 当前值 |
| --- | --- |
| 左轮电机 ID | 1 |
| 右轮电机 ID | 2 |
| 左轮方向 | +1 |
| 右轮方向 | -1 |
| 轮半径 | 0.055 m |
| 控制频率 | 100 Hz |
| 单轮力矩限制 | 0.10 N·m |
| 倾倒阈值 | 相对启用姿态 15° |
| 电机反馈超时 | 100 ms |
| IMU 超时 | 30 ms |
| 等待反馈超时 | 2 s |

控制频率当前设为 100 Hz，因为 BMI088 现有加速度计和陀螺仪配置均为 100 Hz。提升到 500 Hz 前必须先同步修改 IMU ODR、滤波带宽和超时阈值。

## 故障位

| 位 | 含义 |
| --- | --- |
| `0x01` | IMU 无效或超时 |
| `0x02` | 左轮反馈无效或超时 |
| `0x04` | 右轮反馈无效或超时 |
| `0x08` | CAN 初始化、总线或发送故障 |
| `0x10` | 超过倾倒角 |
| `0x20` | 控制周期严重超时 |
| `0x40` | 启用后等待电机反馈超时 |
| `0x80` | 参数或控制器配置错误 |

## 构建结果

从核固件已生成：

```text
output/target/lib/firmware/openamp_core0.elf
```

文件为静态链接 AArch64 ELF，入口地址为 `0xb0100000`。

主核测试程序：

```text
/home/cnvhk/phytium-work/openamp_rpmsg_comm/build/rpmsg_client
```

当前开发机上的该二进制为 x86_64，仅用于协议和编译检查。把源码放到飞腾派后执行：

```bash
cd ~/openamp_rpmsg_comm
make client
```

即可生成飞腾派本机可运行的 AArch64 Linux 客户端。

## 实机测试顺序

首次测试必须架空轮子、扶正机身并保持至少 1 秒静止：

```bash
sudo ./build/rpmsg_client /dev/rpmsg0 balance-status
sudo ./build/rpmsg_client /dev/rpmsg0 balance-enable
watch -n 0.1 'sudo ./build/rpmsg_client /dev/rpmsg0 balance-status'
sudo ./build/rpmsg_client /dev/rpmsg0 balance-disable
```

`balance-enable` 会先做 1 秒陀螺零偏校准，再切换 ID 1/2 到力矩模式并等待新反馈。架空时向前轻微倾斜，两个轮子在地面接触方向上都应向前追赶；方向不对时立即执行 `balance-disable`，先修正方向配置，不修改 LQR 增益。

## 实机前必须确认

- ID 1/2 是否确实为左右轮，且右轮镜像方向是否为 `-1`。
- BMI088 绕轮轴是否为 gyro Y，pitch 正方向是否正确。
- 电机反馈位置是否为 `0.01°`、速度是否为 rpm、电流是否为 `0.01 A`。
- 力矩寄存器 `0x0020` 的有符号值和 `0.01 N·m` 比例是否与实际电机固件一致。
- 55 mm 是否为负载后的有效轮半径。
- 当前 K 来自简化模型，只能用于低力矩架空方向验证；落地前应替换为实测模型增益并逐级提高限幅。
- 当前没有已知 GPIO 映射可接物理急停，软件急停入口为 `balance-disable`，其余自动保护由从核状态机执行。

## 源码位置

实际持久化构建源位于：

```text
package/phytium-standalone/openamp_app/
```

Buildroot 补丁和外设配置位于：

```text
package/phytium-standalone/0001-openamp-enable-application-peripherals.patch
package/phytium-standalone/0002-openamp-connect-application-rpmsg-handler.patch
```

可移植业务副本位于：

```text
/home/cnvhk/phytium-work/openamp_rpmsg_comm/remote_firmware/
```
