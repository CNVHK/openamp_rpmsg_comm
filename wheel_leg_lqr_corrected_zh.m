clear;
clc;
close all;

%% 固定腿高轮腿机器人：四状态离散 LQR 中文模板
%
% 状态向量：
%   x = [theta; theta_dot; position; velocity]
%
%   theta      车身俯仰角，rad
%   theta_dot  车身俯仰角速度，rad/s
%   position   轮轴水平位置，m
%   velocity   轮轴水平速度，m/s
%
% 控制输入：
%   tau_total = tau_left + tau_right，单位 N*m
%
% 正方向约定：
%   theta > 0       表示车身向前倾
%   position > 0    表示机器人向前移动
%   tau_total > 0   表示轮子驱动机器人向前
%
% 电机既向轮子施加驱动力矩，也向车身施加大小相等、方向相反的
% 反作用力矩。线性化方程为：
%
%   a*theta_ddot + b*position_ddot - M*g*l*theta = -tau_total
%   b*theta_ddot + c*position_ddot               =  tau_total/r
%
% 本模型只适用于腿部相对车身完全固定的第一阶段。如果腿部关节参与
% 运动，需要重新建立至少六状态的轮腿模型，不能直接沿用本增益。

%% 1. 机械参数

m_total = 6.95;       % 整车总质量，kg
m = 0.19;             % 两个旋转轮组的总质量，kg
M = m_total - m;      % 除旋转轮组外的固定车身质量，kg

r = 0.03225;          % 车轮有效滚动半径，m
l = 0.073;            % 车身质心到轮轴的垂直距离，m
I = 0.025;            % 车身绕质心、平行轮轴方向的俯仰惯量，kg*m^2
g = 9.81;             % 重力加速度，m/s^2

% 将两个轮组近似为实心圆盘。后续有 CAD 或实测结果时应替换此值。
Iw = 0.5 * m * r^2;   % 两个轮组的总转动惯量，kg*m^2

% 必须与从核固件中的 BALANCE_CONTROL_HZ 一致。
control_hz = 100;
Ts = 1 / control_hz;

% 当前电机协议和固件限制。
tau_single_limit = 0.22;       % 单轮最大允许力矩，N*m
tau_total_limit = 2 * tau_single_limit;
tau_single_resolution = 0.01;  % 单轮力矩命令分辨率，N*m

fprintf('================ 机械参数 ================\n');
fprintf('整车总质量：                 %.6f kg\n', m_total);
fprintf('固定车身质量 M：             %.6f kg\n', M);
fprintf('两个旋转轮组总质量 m：       %.6f kg\n', m);
fprintf('质量合计 M+m：               %.6f kg\n', M + m);
fprintf('车轮有效半径 r：             %.8f m\n', r);
fprintf('质心高度 l：                 %.8f m\n', l);
fprintf('车身俯仰惯量 I：             %.9f kg*m^2\n', I);
fprintf('两个轮组总转动惯量 Iw：      %.9f kg*m^2\n', Iw);
fprintf('控制周期 Ts：                %.6f s (%d Hz)\n', Ts, control_hz);
fprintf('单轮力矩限制：               %.6f N*m\n', tau_single_limit);
fprintf('单轮力矩分辨率：             %.6f N*m\n', tau_single_resolution);

mass_error = abs(m_total - M - m);
if mass_error > 1.0e-9
    error('质量检查失败：m_total 与 M+m 不一致。');
end

if tau_single_limit <= 0 || tau_single_resolution <= 0
    error('力矩限制和力矩分辨率必须大于零。');
end

% 仅用于理解当前执行器能力，不代表动态过程中一定能在该角度恢复。
static_ratio = tau_total_limit / (M * g * l);
if static_ratio < 1
    static_balance_angle_deg = asind(static_ratio);
    fprintf('当前总力矩对应的静态重力矩角度约为：%.3f deg\n', ...
            static_balance_angle_deg);
else
    static_balance_angle_deg = 90;
    fprintf('当前总力矩大于线性模型的最大静态重力矩。\n');
end

fprintf('机械参数检查通过。\n');

%% 2. 建立修正后的连续状态空间模型

a = I + M * l^2;
b = M * l;
c = M + m + Iw / r^2;
den = a * c - b^2;

if den <= 0
    error('模型分母 den <= 0，请检查 M、m、l、I、Iw 和 r。');
end

% 状态顺序：x = [theta; theta_dot; position; velocity]
A = [0,                 1, 0, 0;
     c*M*g*l/den,       0, 0, 0;
     0,                 0, 0, 1;
    -b*M*g*l/den,       0, 0, 0];

% 输入为左右电机力矩之和 tau_total，而不是外部水平推力。
B_tau = [0;
        -(c + b/r)/den;
         0;
         (b + a/r)/den];

% 只用于和旧的“等效水平力”模型比较，不要将该增益写入当前固件。
B_force = B_tau * r;

C = eye(4);
D_tau = zeros(4, 1);

fprintf('\n================ 连续状态空间模型 ================\n');
disp('A =');
disp(A);
disp('B_tau（输入为左右轮总力矩，N*m）=');
disp(B_tau);
disp('B_force（仅用于与旧模型比较）=');
disp(B_force);

continuous_rank = rank(ctrb(A, B_tau));
fprintf('连续系统可控性矩阵秩：%d / 4\n', continuous_rank);
if continuous_rank ~= 4
    error('连续系统不可控，不能继续设计 LQR。');
end

%% 3. 按实际控制周期离散化

sys_c = ss(A, B_tau, C, D_tau);
sys_d = c2d(sys_c, Ts, 'zoh');
Ad = sys_d.A;
Bd = sys_d.B;

discrete_rank = rank(ctrb(Ad, Bd));
fprintf('\n================ 离散模型 ================\n');
fprintf('离散系统可控性矩阵秩：%d / 4\n', discrete_rank);
if discrete_rank ~= 4
    error('离散系统不可控，不能继续设计 LQR。');
end

%% 4. LQR 调参区
%
% q_theta 增大：更重视倾角，更早产生回正力矩。
% q_theta_dot 增大：更重视倾倒速度，提高阻尼和接住速度。
% q_position 增大：更努力返回启动位置，但可能干扰姿态优先级。
% q_velocity 增大：更强地限制行驶速度，但可能妨碍轮子快速接车身。
% R_scale 减小：整体控制更强；R_scale 增大：整体控制更柔和。
%
% 每轮实机测试只修改一组参数，并保留力矩、倾角和轮速保护。

q_theta = 1200;
q_theta_dot = 80;
q_position = 0.25;
q_velocity = 1.25;

Q = diag([q_theta, q_theta_dot, q_position, q_velocity]);

% 旧模型使用 R_force=0.5。先换算为总力矩输入的名义惩罚，再通过
% R_scale 调节整体控制强度。
R_force = 0.5;
R_tau_nominal = R_force / r^2;
R_scale = 0.5;
R_tau = R_tau_nominal * R_scale;

fprintf('\n================ LQR 调参设置 ================\n');
fprintf('Q = diag([%.6f, %.6f, %.6f, %.6f])\n', ...
        q_theta, q_theta_dot, q_position, q_velocity);
fprintf('R_tau_nominal = %.9f\n', R_tau_nominal);
fprintf('R_scale       = %.9f\n', R_scale);
fprintf('R_tau         = %.9f\n', R_tau);

if any(diag(Q) <= 0) || R_tau <= 0
    error('Q 的对角元素和 R_tau 必须全部大于零。');
end

%% 5. 计算离散 LQR 增益

K_tau = dlqr(Ad, Bd, Q, R_tau);
Acl_d = Ad - Bd * K_tau;
poles_d = eig(Acl_d);

% 仅用于旧模型对比。当前 OpenAMP 固件直接使用 K_tau。
K_force_equivalent = K_tau / r;

fprintf('\n================ 离散 LQR 结果 ================\n');
disp('K_tau（输出为左右轮总力矩，N*m）=');
disp(K_tau);
disp('离散闭环极点 eig(Ad-Bd*K_tau) =');
disp(poles_d);
disp('离散闭环极点模长 =');
disp(abs(poles_d));

if any(abs(poles_d) >= 1)
    error('存在模长不小于1的离散闭环极点，禁止部署该增益。');
end

fprintf('\n写入当前 OpenAMP 固件的直接总力矩增益：\n');
fprintf('K1 = %.9ff;\n', K_tau(1));
fprintf('K2 = %.9ff;\n', K_tau(2));
fprintf('K3 = %.9ff;\n', K_tau(3));
fprintf('K4 = %.9ff;\n', K_tau(4));
fprintf(['tau_total = -(K1*theta + K2*theta_dot + ' ...
         'K3*position + K4*velocity)\n']);
fprintf(['tau_left=tau_total/2，tau_right=tau_total/2，' ...
         '之后再做左右电机安装方向映射。\n']);
fprintf('注意：当前固件只复制 K_tau，不要复制 K_force_equivalent。\n');

%% 6. 带限幅、量化和一拍延迟的离散仿真

simulation_time = 5.0;
time = 0:Ts:simulation_time;
sample_count = numel(time);

% 自动测试多个初始倾角。线性仿真只能筛除明显不合理的参数，不能代替
% 有保护绳的实机测试。
initial_pitch_tests_deg = [2.0, 3.0, 5.0];
test_count = numel(initial_pitch_tests_deg);

use_torque_quantization = true;
use_one_sample_delay = true;

simulation_results = repmat(struct( ...
    'initial_pitch_deg', 0, ...
    'x', [], ...
    'tau_command', [], ...
    'tau_applied', [], ...
    'tau_single', [], ...
    'saturated', [], ...
    'peak_total_torque', 0, ...
    'peak_single_torque', 0, ...
    'saturated_ratio_percent', 0, ...
    'final_pitch_deg', 0, ...
    'final_position_m', 0), 1, test_count);

fprintf('\n================ 带约束离散仿真 ================\n');

for test_index = 1:test_count
    theta_initial_deg = initial_pitch_tests_deg(test_index);

    x = zeros(4, sample_count);
    x(:, 1) = [deg2rad(theta_initial_deg); 0; 0; 0];
    x_reference = zeros(4, 1);

    tau_command = zeros(1, sample_count);
    tau_applied = zeros(1, sample_count);
    tau_single = zeros(1, sample_count);
    saturated = false(1, sample_count);

    for k = 1:(sample_count - 1)
        tau_raw = -K_tau * (x(:, k) - x_reference);
        tau_limited = min(max(tau_raw, -tau_total_limit), ...
                          tau_total_limit);
        saturated(k) = abs(tau_raw) > tau_total_limit;

        single_command = tau_limited / 2;

        if use_torque_quantization
            single_command = ...
                round(single_command / tau_single_resolution) * ...
                tau_single_resolution;
        end

        single_command = min(max(single_command, -tau_single_limit), ...
                             tau_single_limit);

        tau_single(k) = single_command;
        tau_command(k) = 2 * single_command;

        if use_one_sample_delay && k > 1
            tau_applied(k) = tau_command(k - 1);
        elseif ~use_one_sample_delay
            tau_applied(k) = tau_command(k);
        end

        x(:, k + 1) = Ad * x(:, k) + Bd * tau_applied(k);
    end

    tau_single(end) = tau_single(end - 1);
    tau_command(end) = tau_command(end - 1);
    tau_applied(end) = tau_applied(end - 1);
    saturated(end) = saturated(end - 1);

    peak_total_torque = max(abs(tau_command));
    peak_single_torque = max(abs(tau_single));
    saturated_ratio_percent = 100 * nnz(saturated) / sample_count;
    final_pitch_deg = rad2deg(x(1, end));
    final_position_m = x(3, end);

    simulation_results(test_index).initial_pitch_deg = theta_initial_deg;
    simulation_results(test_index).x = x;
    simulation_results(test_index).tau_command = tau_command;
    simulation_results(test_index).tau_applied = tau_applied;
    simulation_results(test_index).tau_single = tau_single;
    simulation_results(test_index).saturated = saturated;
    simulation_results(test_index).peak_total_torque = peak_total_torque;
    simulation_results(test_index).peak_single_torque = peak_single_torque;
    simulation_results(test_index).saturated_ratio_percent = ...
        saturated_ratio_percent;
    simulation_results(test_index).final_pitch_deg = final_pitch_deg;
    simulation_results(test_index).final_position_m = final_position_m;

    fprintf('\n初始倾角：%.3f deg\n', theta_initial_deg);
    fprintf('峰值总力矩命令：          %.6f N*m\n', peak_total_torque);
    fprintf('峰值单轮力矩命令：        %.6f N*m\n', peak_single_torque);
    fprintf('力矩饱和采样比例：        %.3f %%\n', ...
            saturated_ratio_percent);
    fprintf('仿真结束俯仰角：          %.6f deg\n', final_pitch_deg);
    fprintf('仿真结束水平位置：        %.6f m\n', final_position_m);
    fprintf('仿真最大绝对俯仰角：      %.6f deg\n', ...
            max(abs(rad2deg(x(1, :)))));

    if max(abs(x(1, :))) > deg2rad(15)
        warning('初始 %.1f deg 仿真超过15度，禁止直接部署该增益。', ...
                theta_initial_deg);
    end

    if abs(x(1, end)) > deg2rad(0.5)
        warning('初始 %.1f deg 仿真结束时没有回到0.5度以内。', ...
                theta_initial_deg);
    end
end

%% 7. 绘制三组仿真结果

figure('Name', '固定腿高轮腿机器人离散 LQR 仿真');

subplot(2, 2, 1);
hold on;
for test_index = 1:test_count
    plot(time, ...
         rad2deg(simulation_results(test_index).x(1, :)), ...
         'LineWidth', 1.3, ...
         'DisplayName', sprintf('初始 %.1f deg', ...
                                initial_pitch_tests_deg(test_index)));
end
grid on;
xlabel('时间 / s');
ylabel('俯仰角 / deg');
title('车身俯仰角');
legend('Location', 'best');

subplot(2, 2, 2);
hold on;
for test_index = 1:test_count
    plot(time, ...
         simulation_results(test_index).x(2, :), ...
         'LineWidth', 1.3, ...
         'DisplayName', sprintf('初始 %.1f deg', ...
                                initial_pitch_tests_deg(test_index)));
end
grid on;
xlabel('时间 / s');
ylabel('俯仰角速度 / rad/s');
title('俯仰角速度');
legend('Location', 'best');

subplot(2, 2, 3);
hold on;
for test_index = 1:test_count
    plot(time, ...
         simulation_results(test_index).x(3, :), ...
         'LineWidth', 1.3, ...
         'DisplayName', sprintf('初始 %.1f deg', ...
                                initial_pitch_tests_deg(test_index)));
end
grid on;
xlabel('时间 / s');
ylabel('轮轴位置 / m');
title('轮轴水平位置');
legend('Location', 'best');

subplot(2, 2, 4);
hold on;
for test_index = 1:test_count
    plot(time, ...
         simulation_results(test_index).tau_single, ...
         'LineWidth', 1.3, ...
         'DisplayName', sprintf('初始 %.1f deg', ...
                                initial_pitch_tests_deg(test_index)));
end
yline(tau_single_limit, '--r', '正力矩限制');
yline(-tau_single_limit, '--r', '负力矩限制');
grid on;
xlabel('时间 / s');
ylabel('单轮力矩 / N*m');
title('单轮力矩命令');
legend('Location', 'best');

%% 8. 保存结果

save('wheel_leg_lqr_corrected_zh_result.mat', ...
     'M', 'm', 'm_total', 'r', 'l', 'I', 'Iw', 'g', ...
     'control_hz', 'Ts', ...
     'A', 'B_tau', 'B_force', 'Ad', 'Bd', ...
     'Q', 'q_theta', 'q_theta_dot', 'q_position', 'q_velocity', ...
     'R_force', 'R_tau_nominal', 'R_scale', 'R_tau', ...
     'K_tau', 'K_force_equivalent', 'poles_d', ...
     'tau_single_limit', 'tau_total_limit', ...
     'tau_single_resolution', 'static_balance_angle_deg', ...
     'initial_pitch_tests_deg', 'simulation_results');

fprintf('\n结果已保存到：wheel_leg_lqr_corrected_zh_result.mat\n');
fprintf('部署到当前固件时只复制 K_tau 的四个增益。\n');
