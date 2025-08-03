function [up_state, up_P] = localization_ekf_update(state, P, y, R)
% LOCALIZATION_EKF_UPDATE  扩展卡尔曼滤波器（EKF）的测量更新步骤
%
% 语法:
%   [up_state, up_P] = localization_ekf_update(state, P, y, R)
%
% 输入参数:
%   state - 当前预测状态（结构体），包含：
%           .Rot: 2x2 旋转矩阵（方向，属于 SO(2)）
%           .p:   2x1 位置向量 [x; y]
%   P     - 当前预测协方差矩阵（3x3），表示状态不确定性
%   y     - 实际观测值（2x1 向量），通常是 GPS 测量 [x_gps; y_gps]
%   R     - 观测噪声协方差矩阵（2x2），通常为 diag(gps_noise_std^2)
%
% 输出参数:
%   up_state - 更新后的状态（结构体）
%   up_P     - 更新后的协方差矩阵（3x3）
%
% 功能说明：
%   本函数实现 EKF 的标准更新流程：
%     1. 线性化观测模型，得到雅可比矩阵 H
%     2. 计算卡尔曼增益 K
%     3. 计算观测残差（创新量）并用于修正状态
%     4. 更新状态和协方差
%
% 注意：
%   本实现假设观测仅依赖于位置（y = h(state) = p），与方向无关。
%   状态更新使用了一个局部误差反馈机制（通过 phi 函数），以保持几何一致性。

%% 构建观测模型雅可比矩阵 H ≈ ∂h/∂x
% 观测函数 h(state) = state.p（即只观测位置）
% 因此其对状态的偏导数为：
%   H = [∂h/∂θ, ∂h/∂x, ∂h/∂y] = [0, 1, 0;
%                                   0, 0, 1]
% 即：观测仅与位置有关，与航向角无关
H = [zeros(2, 1), eye(2)];  % H 是 2×3 矩阵：[0 1 0; 0 0 1]

%% 计算测量残差的协方差（创新协方差）
% S = H*P*H' + R
% 表示“预测观测值的不确定性”，用于归一化卡尔曼增益
S = H * P * H' + R;  % 2x2 矩阵

%% 计算卡尔曼增益
% K = P * H' * S^{-1}
% 使用右除 / 等价于 K = P*H'*inv(S)，MATLAB 自动优化求解
K = P * H' / S;  % K 是 3×2 矩阵：将 2D 观测残差映射回 3D 状态空间

%% 计算创新量（innovation）并映射为状态修正量
% 首先计算观测残差：y - h(state)
% 其中 localization_h(state) 应返回 state.p（当前位置预测）
innovation_residual = y - localization_h(state);

% 将残差通过卡尔曼增益投影为状态空间的修正量 ξ（3×1 向量）
xi = K * innovation_residual;

% 注意：xi 是在“误差状态空间”中的修正量，形式为 [δθ; δx; δy]

%% 使用状态更新函数 phi 进行几何一致的修正
% 直接加法会破坏旋转矩阵的正交性（如 Rot + δRot 可能不再是 SO(2)）
% 因此使用 phi 函数进行流形上的更新：
%   new_state = phi(state, xi)
% 其中 phi 可能定义为：
%   - 方向更新：Rot ← Rot * exp(δθ)   （李群指数映射）
%   - 位置更新：p ← p + [δx; δy]
up_state = localization_phi(state, xi);

%% 更新协方差矩阵（Joseph 形式的一种简化）
% 标准公式：P ← (I - K*H) * P
% 该形式保证协方差矩阵对称半正定（在数值稳定时成立）
up_P = (eye(3) - K * H) * P;

% 注意：更鲁棒的形式是 up_P = (I - K*H) * P * (I - K*H)' + K*R*K'
% 但此处使用简化形式，假设数值稳定

% 函数结束
end