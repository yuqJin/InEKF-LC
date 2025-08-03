function [ys, one_hot_ys] = localization_simu_h(states, T, odo_freq, ...
    gps_freq, gps_noise_std)
% LOCALIZATION_SIMU_H 生成带有噪声的GPS测量数据（仿真观测）
%
% 语法:
%   [ys, one_hot_ys] = localization_simu_h(states, T, odo_freq, gps_freq, gps_noise_std)
%
% 输入参数:
%   states         - 真实状态轨迹（结构体数组），每个元素包含 .p (位置) 和 .Rot (方向)
%   T              - 总仿真时间（秒）
%   odo_freq       - 里程计采样频率（Hz），即每秒采集多少次运动数据
%   gps_freq       - GPS 测量频率（Hz），即每秒获取一次位置观测
%   gps_noise_std  - GPS 测量噪声的标准差（单位：米），假设为高斯白噪声
%
% 输出参数:
%   ys             - 带噪声的GPS测量集合，大小为 2×K，每列是一个二维位置观测 [x; y]
%   one_hot_ys     - 二值向量（0/1），标记每个时间步是否有GPS测量（1表示有）
%
% 说明：
%   该函数模拟了低频GPS传感器在高频里程计系统中的观测行为。
%   例如：里程计100Hz，GPS 1Hz，则每100个时间步才有一个GPS读数。
%   测量值 = 真实位置 + 高斯噪声

%% 计算总时间步数
% 根据总时间 T 和里程计频率 odo_freq，计算总的离散时间步数
N = odo_freq * T;

%% 构建“one-hot”测量标记向量
% one_hot_ys(n) == 1 表示第 n 个时间步有 GPS 测量，否则为 0
one_hot_ys = zeros(N, 1);

% 按照 GPS 频率插入测量标志
% 例如：odo_freq=100, gps_freq=1 → 每 100 步一次测量 → 在 1, 101, 201,... 处置1
% 要求 odo_freq / gps_freq 必须是整数（确保测量周期对齐）
one_hot_ys(1:odo_freq/gps_freq:end) = 1;

% 提取所有存在测量的时间步索引（即 one_hot_ys 中值为1的位置）
idxs = find(one_hot_ys); % 得到 [1, 101, 201, ...] 类似的索引

%% 初始化测量数据
% K: 总共会有多少次 GPS 测量
K = length(idxs);

% ys 用于存储所有带噪声的测量值，每列对应一次测量
ys = zeros(2, K); % 每列是 [x; y] 的带噪GPS读数

% k 是测量编号（从1到K），用于填充 ys 的列
k = 1;
for n = 1:K
    % 获取第 idxs(n) 时刻的真实位置（即 states(idxs(n)).p）
    % 添加均值为0、标准差为 gps_noise_std 的高斯噪声
    ys(:, k) = states(idxs(n)).p + gps_noise_std * randn(2, 1);
    
    % 更新测量列索引
    k = k + 1;
end

% 函数结束
end