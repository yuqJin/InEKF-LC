%% 2D Robot Localization 
%% 初始化环境
% 清除工作区所有变量和图形窗口，确保运行环境干净
clear all;
close all;

%% 系统模型定义

% 设置蒙特卡洛仿真实验次数（用于统计性能）
N_mc = 5;

%%
% 定义机器人定位问题中的关键参数
% 总仿真时间（秒）
T = 60; 
% 里程计采样频率（Hz），即每秒采集100次运动数据
odo_freq = 100; 
% 里程计噪声标准差：
%   - 纵向速度噪声（m/s）
%   - 横向滑移速度噪声（m/s）
%   - 角速度（陀螺仪）噪声（rad/s），转换为弧度（1度 = π/180 弧度）
odo_noise_std = [0.1;               % 纵向速度噪声
                 0.1;               % 横向速度噪声
                 1*pi/180];         % 角速度噪声（约1度/秒）

% GPS 定位频率（Hz），每秒获取一次位置测量
gps_freq = 1;
% GPS 测量噪声标准差（米）
gps_noise_std = 0.2;
% 设定机器人期望运行的圆形轨迹半径（单位：米）
radius = 15;
% 注意：此处 radius=30 被注释掉，当前使用的是 15 米

% 总时间步数（根据里程计频率计算）
N = T * odo_freq;
% 数值积分的时间步长（秒）
dt = 1 / odo_freq;

%% 生成真实状态与传感器输入（仿真阶段）

% 调用函数生成真实状态轨迹 states 和控制输入 omegas
% 输入参数包括总时间、里程计频率、噪声水平和轨迹半径
[states, omegas] = localization_gen(T, odo_freq, odo_noise_std, radius);

% 注：states 和 omegas 是结构体数组
% 可通过 states(n) 获取第 n 个时刻的状态，例如：
%   states(n).Rot   -> 2D 旋转矩阵（表示朝向，属于 SO(2)）
%   states(n).p     -> 2D 位置向量 [x; y]
%   omegas(n).v     -> 前向线速度
%   omegas(n).gyro  -> 角速度（来自陀螺仪）

% 使用真实状态生成带有噪声的观测数据（模拟 GPS 和里程计读数）
% ys: 包含带噪声的 GPS 位置测量值
% one_hot_ys: 标记哪些时间点有 GPS 测量（1 表示有，0 表示无）
[ys, one_hot_ys] = localization_simu_h(states, T, odo_freq, gps_freq, ...
    gps_noise_std);

%% 滤波器设计与初始化

% 构建过程噪声协方差矩阵 Q（对角阵，各噪声分量独立）
Q = diag(odo_noise_std.^2);

% 构建观测噪声协方差矩阵 R（GPS 位置噪声，x 和 y 方向独立）
R = gps_noise_std^2 * eye(2);

% 初始协方差矩阵 P0，表示初始估计的不确定性
% 当前设为零矩阵，意味着初始估计非常自信（但实际存在方向误差）
P0 = zeros(3, 3); 

% 可选：若想加入初始方向不确定性（如 3° 或 5°），可取消下面注释
% P0(1,1) = (3*pi/180)^2;  % 初始航向角方差
% P0(2,2) = 0.05;          % 初始 x 位置方差
% P0(3,3) = 0.05;          % 初始 y 位置方差

% Unscented Kalman Filter (UKF) 的缩放参数 alpha
% 控制 sigma 点的分布范围，通常取 1e-3 到 1 之间的小值
alpha = [1e-3, 1e-3, 1e-3];

% 定义 UKF 所需的非线性函数句柄：
%   f: 状态传播函数（预测模型）
%   h: 观测函数（从状态到测量的映射）
%   phi: 局部坐标变换（流形上的误差表示）
%   phi_inv: phi 的逆变换
f = @localization_f;
h = @localization_h;
phi = @localization_phi;
phi_inv = @localization_phi_inv;

% 计算 UKF 所需的权重参数（维度为 3 的状态空间，参数 alpha 给定）
weights = ukf_set_weight(3, 2, alpha);

% 预先计算 Q 的 Cholesky 分解，用于高效生成 sigma 点扰动
cholQ = chol(Q);

%%
% 初始化用于存储不同滤波器误差的数组（Monte-Carlo 多次运行）
% 维度：[误差类型, 时间步数, 蒙特卡洛次数]
% 误差类型：[航向角误差, x误差, y误差]

ukf_errs = zeros(3, N, N_mc);   % UKF-M（Unscented Kalman Filter on Manifold）
ekf_errs = zeros(size(ukf_errs)); % EKF（标准扩展卡尔曼滤波）
iekf_errs = zeros(size(ukf_errs)); % Invariant EKF（不变性扩展卡尔曼滤波）
iekflc_errs = zeros(size(ukf_errs)); % InEKF-LC（带回环校正的不变EKF）

% 开始蒙特卡洛循环
for n_mc = 1:N_mc
    disp("Monte-Carlo iteration(s): " + num2str(n_mc) + "/" + num2str(N_mc));
    
    %%
    % 所有滤波器从真实初始状态开始，但人为加入一个小的方向误差（约1°）
    ekf_state = states(1);     % EKF 初始状态
    iekf_state = states(1);    % InEKF 初始状态
    iekflc_state = states(1);  % InEKF-LC 初始状态
    ukf_state = states(1);     % UKF 初始状态

    % 在初始方向上添加一个随机小误差（通过 SO(2) 指数映射实现）
    % sqrt(P0(1,1)) 表示标准差，若 P0(1,1)>0 则引入不确定性
    ukf_state.Rot = ukf_state.Rot * so2_exp(sqrt(P0(1, 1)));
    
    % 初始化各滤波器的协方差矩阵
    ukf_P = P0;
    iekf_P = P0;
    ekf_P = P0;
    iekflc_P = P0;
    
    % 用于记录整个轨迹上的估计状态（每一步都保存）
    ukf_states = ukf_state;
    iekf_states = iekf_state;
    ekf_states = ekf_state;
    iekflc_states = iekflc_state;

    % 用于记录协方差矩阵随时间变化（用于分析收敛性）
    ukf_Ps = zeros(N, length(ukf_P), length(ukf_P));
    ukf_Ps(1, :, :) = ukf_P;
    iekf_Ps = zeros(N, length(iekf_P), length(iekf_P));
    iekf_Ps(1, :, :) = iekf_P;
    ekf_Ps = zeros(N, length(ekf_P), length(ekf_P));
    ekf_Ps(1, :, :) = ekf_P;
    iekflc_Ps = zeros(N, length(iekflc_P), length(iekflc_P));
    iekflc_Ps(1, :, :) = iekflc_P;
    
    %% 滤波主循环（时间推进）

    % GPS 测量的索引（ys(:,k) 存储第 k 次 GPS 数据）
    k = 2;  % 第一个测量对应 n=1，但 ys(:,1) 已用于初始值？

    % 从第2个时间步开始进行递推滤波
    for n = 2:N
        
        % ------------------ 预测步（Propagation / Time Update）------------------
        
        % 使用 EKF 进行状态预测（基于运动模型和协方差传播）
        [ekf_state, ekf_P] = localization_ekf_propagation(...
            ekf_state, ekf_P, omegas(n-1), dt, Q);
        
        % 使用 Invariant EKF (InEKF) 进行预测（保持几何结构不变性）
        [iekf_state, iekf_P] = localization_iekf_propagation(...
            iekf_state, iekf_P, omegas(n-1), dt, Q);
        
        % InEKF-LC 使用相同的传播步骤（LC 仅影响更新）
        [iekflc_state, iekflc_P] = localization_iekf_propagation(...
            iekflc_state, iekflc_P, omegas(n-1), dt, Q);

        % 使用 UKF 进行预测（通过 sigma 点非线性传播）
        [ukf_state, ukf_P] = ukf_propagation(...
            ukf_state, ukf_P, omegas(n-1), f, dt, phi, phi_inv, cholQ, weights);     

        % ------------------ 更新步（Measurement Update）------------------
        % 仅当当前时刻有 GPS 测量时才执行更新
        if one_hot_ys(n) == 1
            
            % EKF 更新：使用线性化观测模型
            [ekf_state, ekf_P] = localization_ekf_update(...
               ekf_state, ekf_P, ys(:, k), R);
            
            % InEKF 更新：基于李群不变性的更新
            [iekf_state, iekf_P] = localization_iekf_update(...
               iekf_state, iekf_P, ys(:, k), R);
            
            % InEKF-LC 更新：带局部约束（Local Correction）的不变更新
            [iekflc_state, iekflc_P] = localization_iekf_update_LC(...
               iekflc_state, iekflc_P, ys(:, k), R, omegas(n-1), dt);

            % UKF 更新：使用观测函数 h 和权重进行无迹更新
            [ukf_state, ukf_P] = ukf_update(...
                ukf_state, ukf_P, ys(:, k), h, phi, R, weights);
   
            % 更新测量索引
            k = k + 1;
        end
        
        % ------------------ 保存当前时刻的估计结果 ------------------
        ukf_states(n) = ukf_state;
        ukf_Ps(n, :, :) = ukf_P;
        ekf_states(n) = ekf_state;
        iekf_states(n) = iekf_state;
        ekf_Ps(n, :, :) = ekf_P;
        iekf_Ps(n, :, :) = iekf_P;
        iekflc_states(n) = iekflc_state;
        iekflc_Ps(n, :, :) = iekflc_P;
        
    end
    
    % 提取所有时刻的真实状态（旋转矩阵 Rots 和位置 ps）
    [Rots, ps] = localization_get_states(states);
    
    % 提取各滤波器的估计状态
    [ukf_Rots, ukf_ps] = localization_get_states(ukf_states);   % UKF-M
    [iekf_Rots, iekf_ps] = localization_get_states(iekf_states); % InEKF
    [ekf_Rots, ekf_ps] = localization_get_states(ekf_states);   % EKF
    [iekflc_Rots, iekflc_ps] = localization_get_states(iekflc_states); % InEKF-LC

    % 计算各滤波器在本次蒙特卡洛运行中的误差
    % localization_errors 输出格式：[方向误差; dx; dy]
    ukf_errs(:, :, n_mc) = localization_errors(Rots, ukf_Rots, ps, ukf_ps);
    iekf_errs(:, :, n_mc) = localization_errors(Rots, iekf_Rots, ps, iekf_ps);
    ekf_errs(:, :, n_mc) = localization_errors(Rots, ekf_Rots, ps, ekf_ps);
    iekflc_errs(:, :, n_mc) = localization_errors(Rots, iekflc_Rots, ps, iekflc_ps);
end

%% 结果可视化

% 设置全局字体解释器为 LaTeX，支持数学公式显示
set(groot, 'defaulttextinterpreter', 'latex');  
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');  
set(groot, 'defaultLegendInterpreter', 'latex');

% 生成时间轴（从 0 到 T，共 N 个点）
N = length(ps);
t = linspace(0, dt*N, N);

% ========== 图1：XY 平面上的轨迹对比 ==========
fig1 = figure;
title('Position in $xy$-plan')  % 使用 LaTeX 显示标题
xlabel('$x$ (m)')
ylabel('$y$ (m)')
hold on;
grid on;

% 绘制真实轨迹（黑色实线）
plot(ps(:, 1), ps(:, 2), 'k', 'LineWidth', 1.5);

% 绘制各滤波器估计轨迹
plot(iekf_ps(:, 1), iekf_ps(:, 2), "Color",[0 0.4470 0.7410], 'linewidth',1.5);     % InEKF: 蓝色实线
plot(ekf_ps(:, 1), ekf_ps(:, 2), '--', "Color",[0.8500 0.3250 0.0980], 'linewidth',1.5); % EKF: 橙色虚线
plot(ukf_ps(:, 1), ukf_ps(:, 2), ':', "Color",[0.9290 0.6940 0.1250], 'linewidth',1.5);  % UKF: 黄色点线
plot(iekflc_ps(:, 1), iekflc_ps(:, 2), "Color",[0.3010 0.7450 0.9330], 'linewidth',1.5); % InEKF-LC: 青色实线

% 添加图例
legend('Ground truth', 'EKF-LG', 'InEKF', 'UKF-LG', 'InEKF-LC');
axis equal; % 保持坐标轴比例一致，避免图形变形

% ========== 定义误差统计函数 ==========
% f_rot: 计算平均航向角误差（均方根）
f_rot = @(x) squeeze(sqrt(mean(x(1, :, :).^2, 3)));

% f_p: 计算位置误差的均方根（综合 x 和 y）
f_p = @(x) squeeze(sqrt(mean(x(2, :, :).^2 + x(3, :, :).^2, 3)));

% 提取各滤波器的平均误差曲线（跨蒙特卡洛运行的均值）
ekf_err_Rots = f_rot(ekf_errs);
ekf_err_ps = f_p(ekf_errs);
iekf_err_Rots = f_rot(iekf_errs); 
iekf_err_ps = f_p(iekf_errs);
ukf_err_Rots = f_rot(ukf_errs);
ukf_err_ps = f_p(ukf_errs);
iekflc_err_Rots = f_rot(iekflc_errs);
iekflc_err_ps = f_p(iekflc_errs);   

% ========== 图2：航向角误差随时间变化 ==========
fig2 = figure;
xlabel('$t$ (s)')
ylabel('orientation error (deg)')
hold on;
grid on;
lwdt = 2;

% 绘制各滤波器的航向角误差（弧度转为角度：180/pi）
plot(t, 180/pi * ekf_err_Rots, "Color",[0 0.4470 0.7410], 'linewidth',lwdt);     % EKF
plot(t, 180/pi * iekf_err_Rots, '--', "Color",[0.8500 0.3250 0.0980], 'linewidth',lwdt); % InEKF
plot(t, 180/pi * ukf_err_Rots, ':', "Color",[0.9290 0.6940 0.1250], 'linewidth',lwdt);  % UKF
plot(t, 180/pi * iekflc_err_Rots, "Color",[0.3010 0.7450 0.9330], 'linewidth',lwdt);    % InEKF-LC

legend('EKF-LG','InEKF','UKF-LG','InEKF-LC');

% ========== 图3：位置误差随时间变化 ==========
fig3 = figure;
xlabel('$t$ (s)')
ylabel('position error (m)')
hold on;
grid on;

% 绘制各滤波器的位置均方根误差
plot(t, ekf_err_ps, "Color",[0 0.4470 0.7410], 'linewidth',lwdt);           % EKF
plot(t, iekf_err_ps, '--', "Color",[0.8500 0.3250 0.0980], 'linewidth',lwdt); % InEKF
plot(t, ukf_err_ps, ':', "Color",[0.9290 0.6940 0.1250], 'linewidth',lwdt);  % UKF
plot(t, iekflc_err_ps, '-.', "Color",[0.3010 0.7450 0.9330], 'linewidth',lwdt); % InEKF-LC

legend('EKF-LG','InEKF','UKF-LG','InEKF-LC');