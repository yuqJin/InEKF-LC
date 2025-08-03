function [errors] = localization_errors(Rots, hat_Rots, ps, hat_ps)
% LOCALIZATION_ERRORS  计算姿态（方向）和位置的估计误差
%
% 语法:
%   errors = localization_errors(Rots, hat_Rots, ps, hat_ps)
%
% 输入参数:
%   Rots      - 真实方向轨迹（元胞数组），每个元素是 2x2 旋转矩阵（属于 SO(2)）
%   hat_Rots  - 估计方向轨迹（元胞数组），结构同 Rots
%   ps        - 真实位置轨迹（N×2 矩阵），每行是 [x; y]
%   hat_ps    - 估计位置轨迹（N×2 矩阵），每行是 [x_hat; y_hat]
%
% 输出参数:
%   errors    - 误差矩阵（3×N），每列对应一个时刻的误差，格式为：
%               [方向误差(弧度); x误差; y误差]
%
% 功能说明：
%   该函数计算两个关键误差：
%     1. 方向误差：在 SO(2) 流形上定义的角误差（通过李代数对数映射）
%     2. 位置误差：欧氏空间中的差值（直接相减）
%
% 注意：
%   使用流形方法计算方向误差，避免了角度跳变问题（如 359° vs 1°），
%   是现代状态估计中的标准做法。

%% 获取总时间步数
N = length(Rots);  % 假设 Rots, hat_Rots, ps, hat_ps 都有 N 个时间点

%% 初始化误差矩阵
% errors(1,:) : 方向误差（弧度）
% errors(2,:) : x 方向位置误差
% errors(3,:) : y 方向位置误差
errors = zeros(3, N);

%% 计算每一时刻的方向误差（在 SO(2) 李代数上）
% 对于每个时间步 n，计算真实旋转与估计旋转之间的“相对旋转”：
%   R_error = R_true^T * R_est
% 然后使用 so2_log 将其映射到李代数 so(2)，得到一个标量（即角度误差）
%
% 数学含义：
%   so2_log(Rots{n}' * hat_Rots{n}) 返回的是从真实方向到估计方向的旋转角（弧度）
%   正负号表示顺时针或逆时针偏差
%
% 优势：
%   - 自动处理角度周期性（如 -π 和 π 等价）
%   - 是流形上的测地线距离（最短路径）
for n = 1:N
    errors(1, n) = so2_log(Rots{n}' * hat_Rots{n});
end

%% 计算位置误差（逐点相减）
% 注意：ps 和 hat_ps 是 N×2 矩阵（每行一个时刻的位置）
% 而 errors(2:3,:) 要求是 2×N，因此需要转置
errors(2:3, :) = (ps - hat_ps)';

% 解释：
%   ps - hat_ps 得到 N×2 的位置残差矩阵
%   转置后变为 2×N，正好填充 errors 的第2、3行

% 函数结束
end