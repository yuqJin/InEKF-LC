function [states, omegas] = localization_simu_f(T, odo_freq, odo_noise_std, radius)
% LOCALIZATION_SIMU_F  生成真实轨迹与带噪输入（仿真运动过程）
%
% 语法:
%   [states, omegas] = localization_simu_f(T, odo_freq, odo_noise_std, radius)
%
% 描述:
%   模拟机器人在圆形轨迹上的运动，输出真实状态序列和带噪声的里程计输入。
%   用于生成 EKF/InEKF/UKF 等滤波器的输入数据。
%
% 输入:
%   T              - 总时间（秒）
%   odo_freq       - 里程计频率（Hz）
%   odo_noise_std  - 里程计噪声标准差 [v_long; v_lat; gyro]
%   radius         - 圆形轨迹半径（米）
%
% 输出:
%   states  - 真实状态序列（结构体数组），含 .Rot 和 .p
%   omegas  - 带噪控制输入序列（结构体数组），含 .v 和 .gyro

%% 参数设置
N  = T * odo_freq;        % 总时间步数
dt = 1 / odo_freq;        % 积分步长（秒）

%% 设定名义运动输入（理想圆周运动）
v    = [2*pi*radius/T; 0]; % 前向速度（切向，m/s），横向速度为0
gyro = 2*pi / T;           % 角速度（rad/s），对应周期 T

%% 初始化
w = zeros(3, 1);                    % 过程噪声设为0，生成真实轨迹
omegas(N) = struct;                 % 预分配输入数组
states(N) = struct;                 % 预分配状态数组
states(1).Rot = eye(2);             % 初始方向：x 轴对齐
states(1).p = zeros(2, 1);          % 初始位置：原点

%% 时间推进循环
for n = 2:N
    % 保存无噪声输入（用于真实状态传播）
    omegas(n-1).v    = v;
    omegas(n-1).gyro = gyro;
    
    % 使用无噪声输入传播真实状态
    states(n) = localization_f(states(n-1), omegas(n-1), w, dt);
    
    % 叠加高斯噪声，生成实际观测的里程计输入
    omegas(n-1).v    = omegas(n-1).v + odo_noise_std(1:2) .* randn(2, 1);
    omegas(n-1).gyro = omegas(n-1).gyro + odo_noise_std(3) * randn;
end
end