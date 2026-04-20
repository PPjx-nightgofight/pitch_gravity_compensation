%% =====================================================================
%  Pitch 轴重力补偿标定脚本
%  动力学模型：T = A·cos(θ) + B + sign(dir)·Tf
%    A  = m·g·L    重力矩幅值 (Nm)
%    B  = 常数偏置  来自连接杆支撑力/结构不对称
%    Tf = 库仑摩擦力幅值 (Nm)
%    dir= 运动方向 (+1上扫, -1下扫)
%  使用最小二乘法一次性拟合三个参数
%% =====================================================================
clc; clear; close all;

%% 1. 读取数据
data   = readtable('angle-torque.csv');
angle_deg = data.angle;    % 角度，单位 °
torque    = data.torque;   % 力矩，单位 Nm
angle_rad = angle_deg * pi / 180;

fprintf('共读取 %d 条数据\n', length(angle_deg));

%% 2. 判断运动方向（上扫 / 下扫）
%  差分：相邻角度差 > 0 为上扫，< 0 为下扫
%  第一个点方向与第二个点相同
diff_angle = diff(angle_deg);
direction  = sign([diff_angle(1); diff_angle]);  % +1 或 -1，长度与数据一致

% 去掉方向为 0 的点（静止点，无意义）
valid_idx  = direction ~= 0;
angle_rad  = angle_rad(valid_idx);
torque     = torque(valid_idx);
direction  = direction(valid_idx);

fprintf('有效数据点：%d 条\n', sum(valid_idx));
fprintf('上扫点数：%d，下扫点数：%d\n', sum(direction>0), sum(direction<0));

%% 3. 构建线性回归矩阵
%  T = A·cos(θ) + B·1 + Tf·sign(dir)
%  写成矩阵形式：Φ * [A; B; Tf] = T
%    Φ = [cos(θ),  1,  sign(dir)]
Phi = [cos(angle_rad), sin(angle_rad), ones(size(angle_rad)), direction];


%% 4. 最小二乘求解
%  x = (Φ'Φ)^{-1} Φ' T
 x = Phi \ torque;   % MATLAB 的 \ 即最小二乘解

A1 = x(1);   % cos 系数
A2 = x(2);   % sin 系数
B_fit  = x(3);   % 偏置
Tf_fit = x(4);   % 摩擦力

% 反推物理参数
A_fit = sqrt(A1^2 + A2^2);          % 真实重力矩幅值
phi_deg = atan2(-A2, A1) * 180/pi;    % 零点偏差角（°）

fprintf('\n===== 标定结果 =====\n');
fprintf('真实重力矩幅值 A = %.4f Nm\n', A_fit);
fprintf('零点偏差角 φ   = %.2f °\n', phi_deg);
%fprintf('A  (重力矩幅值 m·g·L) = %.4f Nm\n', A_fit);
fprintf('B  (结构偏置)         = %.4f Nm\n', B_fit);
fprintf('Tf (库仑摩擦力)       = %.4f Nm\n', abs(Tf_fit));

%% 5. 计算拟合残差和 R²
T_pred   = Phi * x;
residual = torque - T_pred;
SS_res   = sum(residual.^2);
SS_tot   = sum((torque - mean(torque)).^2);
R2       = 1 - SS_res / SS_tot;
RMSE     = sqrt(mean(residual.^2));

fprintf('\n===== 拟合质量 =====\n');
fprintf('R²   = %.4f  (越接近1越好)\n', R2);
fprintf('RMSE = %.4f Nm\n', RMSE);

%% 6. 绘图
theta_plot = linspace(-35, 25, 200) * pi / 180;

% 重力曲线（去除摩擦和偏置后的纯重力项）
T_gravity_only = A_fit * cos(theta_plot);

% 带偏置的重力补偿曲线（实际用于代码的前馈）
T_feedforward = A1 * cos(theta_plot) + A2 * sin(theta_plot) + B_fit;
T_pred_up     = T_feedforward + abs(Tf_fit);
T_pred_down   = T_feedforward - abs(Tf_fit);

figure('Name', 'Pitch轴重力补偿标定', 'Position', [100, 100, 1000, 600]);

subplot(2,1,1);
hold on; grid on;
% 原始数据按方向分色绘制
scatter(angle_deg(direction>0), torque(direction>0), 20, 'b', 'filled', ...
    'DisplayName', '上扫数据');
scatter(angle_deg(direction<0), torque(direction<0), 20, 'r', 'filled', ...
    'DisplayName', '下扫数据');
% 拟合曲线
plot(theta_plot*180/pi, T_feedforward, 'g-',  'LineWidth', 2.5, ...
    'DisplayName', sprintf('重力前馈 A=%.3f, B=%.3f', A_fit, B_fit));
plot(theta_plot*180/pi, T_pred_up,    'b--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('预测上扫 (+Tf=%.3f)', abs(Tf_fit)));
plot(theta_plot*180/pi, T_pred_down,  'r--', 'LineWidth', 1.5, ...
    'DisplayName', '预测下扫 (-Tf)');
xlabel('角度 (°)'); ylabel('力矩 (Nm)');
title(sprintf('Pitch轴力矩标定  R²=%.4f  RMSE=%.4f Nm', R2, RMSE));
legend('Location', 'best');

subplot(2,1,2);
hold on; grid on;
plot(residual, 'g.', 'MarkerSize', 8);
yline(0, 'r--', 'LineWidth', 1.5);
yline( RMSE, 'b--', 'DisplayName', sprintf('+RMSE=%.4f', RMSE));
yline(-RMSE, 'b--', 'DisplayName', sprintf('-RMSE=%.4f', RMSE));
xlabel('数据点序号'); ylabel('残差 (Nm)');
title('拟合残差（理想情况下应随机分布在0附近）');
legend('Location', 'best');

%% 7. 输出用于 STM32 代码的宏定义
fprintf('\n===== 复制到 main.c 的宏定义 =====\n');
fprintf('#define GRAVITY_COMP_A1  %.4ff   // cos系数\n', A1);
fprintf('#define GRAVITY_COMP_A2  %.4ff   // sin系数\n', A2);
fprintf('#define GRAVITY_COMP_B   %.4ff   // 结构偏置 (Nm)\n', B_fit);
fprintf('#define COULOMB_FRICTION %.4ff   // 库仑摩擦力 (Nm)\n', abs(Tf_fit));
fprintf('\n// 前馈力矩计算:\n');
fprintf('// float gravity_ff = GRAVITY_COMP_A1 * cosf(pitch_rad)\n');
fprintf('//                  + GRAVITY_COMP_A2 * sinf(pitch_rad)\n');
fprintf('//                  + GRAVITY_COMP_B;\n');

%% 8. 验证：画出去摩擦后的平均曲线对比纯 cos 拟合
figure('Name', '去摩擦验证', 'Position', [200, 200, 800, 400]);

% 注意：angle_deg 和 torque 此时都已经是过滤后的135个点
% 需要用过滤后的变量排序，不能用原始的 angle_deg
angle_deg_valid = angle_deg(valid_idx);   % 过滤后的角度，135个

[angle_sorted, sort_idx] = sort(angle_deg_valid);  % 对过滤后的数据排序
torque_sorted = torque(sort_idx);                  % torque 也是135个，索引匹配

hold on; grid on;
plot(angle_sorted, torque_sorted, 'g.', 'MarkerSize', 6, ...
    'DisplayName', '所有数据点');
plot(theta_plot*180/pi, T_feedforward, 'r-', 'LineWidth', 2.5, ...
    'DisplayName', sprintf('拟合前馈: %.4f·cos+%.4f·sin+%.4f', A1, A2, B_fit));
xlabel('角度 (°)'); ylabel('力矩 (Nm)');
title('实测数据 vs 拟合前馈曲线');
legend('Location', 'best');