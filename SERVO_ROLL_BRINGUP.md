# 四舵机与 IMU Roll 基础验证

本阶段只验证四路 MG996R 接线、命令方向、同步缓动，以及 BMI088 的
roll/roll_rate 融合。不要同时启用轮式平衡，也不要把 roll 直接闭环到腿部。

## 1. 接线与供电

四路 PWM 映射如下：

| 关节命令 | PWM | 飞腾派 40Pin |
| --- | --- | --- |
| servo0 | FPWM1 channel 0 / PWM2_OUT | Pin 32 |
| servo1 | FPWM2 channel 1 / PWM5_OUT | Pin 33 |
| servo2 | FPWM3 channel 0 / PWM6_OUT | Pin 7 |
| servo3 | FPWM3 channel 1 / PWM7_OUT | A43 对应引脚 |

- 四个 MG996R 使用独立的 5–6 V 大电流电源，不能由飞腾派 5 V 引脚供电。
- 舵机电源地和飞腾派地必须共地。
- 首次验证应拆下舵盘或断开连杆。装上连杆后，机器人必须悬空并有刚性支撑。
- `servo-stop` 停止轨迹并保持当前 PWM，不会使舵机失能。卡死时立即切断舵机电源。

## 2. 编译与部署

按项目现有增量流程构建，不需要重建系统镜像：

```bash
make
makeelf
scpelf
reloadrproc
```

确认新客户端版本和从核通信：

```bash
rprun heartbeat
```

客户端版本应为 `0.19.0-leg-servo-roll` 或更高。

## 3. 单独验证 IMU Roll

先停用平衡和云台，把机器人固定并保持完全静止：

```bash
rprun balance-disable
rpgimbal estop
rprun imu-calibrate 100
rprun imu-attitude
```

连续观察：

```bash
watch -n 0.1 'rprun imu-attitude'
```

验证项目：

1. 水平静止时 `roll` 和 `pitch` 应稳定，`roll_rate`、`pitch_rate` 接近 0。
2. 绕 IMU X 轴缓慢左右倾斜时，roll 应连续变化，回到水平后回到原值附近。
3. 静止倾斜时 roll 不能持续漂移；快速倾斜时 roll_rate 应立即响应。
4. 当前定义中，加速度 Y 为正时 roll 为正。若实机左右方向与机器人坐标约定相反，
   修改 `PHYTIUM_BMI088_ROLL_DIRECTION`，不要交换其他 IMU 轴。

`imu-calibrate` 仅允许在平衡和云台均停用时运行。标定过程中必须静止；命令默认采集
100 个样本，约需 1 秒。

## 4. 逐路验证舵机

确保平衡处于 `disabled`：

```bash
rprun balance-status
rprun servo-status
```

以下动作以 90 度为装配中位，只移动 5 度。首次应在舵盘或连杆断开的状态执行：

```bash
rprun servo-move 2000 95 90 90 90
rprun servo-move 2000 90 90 90 90

rprun servo-move 2000 90 95 90 90
rprun servo-move 2000 90 90 90 90

rprun servo-move 2000 90 90 95 90
rprun servo-move 2000 90 90 90 90

rprun servo-move 2000 90 90 90 95
rprun servo-move 2000 90 90 90 90
```

每次记录：实际运动关节、正角方向、是否存在抖动/卡滞、90 度时舵盘机械位置。
`servo-status` 显示的是命令角度和 PWM 脉宽，不是实际位置反馈。

## 5. 原地同步变腿高

完成逐路验证并按 90 度重新安装舵盘后，先用每个关节仅 2–5 度的目标做同步动作。
左右/前后关节的增减方向必须使用上一节的实测结果填写，不能直接假定四路同号。

命令模板：

```bash
rprun servo-move 3000 <s0_deg> <s1_deg> <s2_deg> <s3_deg>
watch -n 0.1 'rprun servo-status'
```

判断通过的条件：

- 四条腿同时开始、同时结束，过程中没有连杆顶死。
- 机身高度变化方向一致，左右高度肉眼无明显差异。
- 支架上的机身 roll 没有持续增大。
- 舵机和电源线不过热，电源没有明显掉压或复位。

当前同步动作是四关节命令插值，还不是并联腿的笛卡尔高度控制。得到五连杆长度、四个
机械零点、每路正方向和安全角范围后，再实现 `(腿高, 前后支撑偏移)` 到四关节角的逆运动学。
在此之前不要落地动态变腿高，也不要同时执行 `balance-enable`。

