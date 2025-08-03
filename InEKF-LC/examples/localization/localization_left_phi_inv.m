function [xi] = localization_left_phi_inv(state, hat_state)
% LOCALIZATION_LEFT_PHI_INV  计算两个 SE(2) 状态之间的左不变误差（对数映射）
%
% 语法:
%   xi = localization_left_phi_inv(state, hat_state)
%
% 描述:
%   本函数计算从“真实状态” state 到“估计状态” hat_state 的**左不变误差**，
%   即：
%       ξ = log( X⁻¹ * X_hat ) ∈ se(2)
%   其中 X 和 X_hat 分别表示真实与估计的位姿（属于 SE(2) 群）。
%
%   该误差 ξ 是一个 3×1 向量 [δθ; δx; δy]，表示：
%     - δθ: 估计航向相对于真实航向的偏差（弧度）
%     - δx, δy: 在机体坐标系下，估计位置相对于真实位置的偏移
%
%   此函数是 localization_left_phi 的逆操作，常用于：
%     - 滤波器误差分析
%     - InEKF / InEKF-LC 的创新量计算
%     - 协方差一致性评估（如 NEES 指标）
%
% 输入参数:
%   state      - 真实或参考状态（结构体），包含：
%                .Rot: 2x2 旋转矩阵（方向，属于 SO(2)）
%                .p:   2x1 位置向量 [x; y]
%   hat_state  - 估计状态（结构体），格式同 state
%
% 输出参数:
%   xi         - 左不变误差向量（3×1），单位：
%                δθ: 弧度
%                δx, δy: 米
%
% 数学原理:
%   1. 将两个状态转换为齐次变换矩阵（3×3）：
%        chi     = [R,    p;    0, 0, 1] ∈ SE(2)
%        hat_chi = [R_hat, p_hat; 0, 0, 1] ∈ SE(2)
%   2. 计算相对变换：Δχ = X⁻¹ * X_hat
%   3. 对 Δχ 进行对数映射：ξ = log(Δχ) ∈ se(2)
%   4. 提取 ξ 对应的 3×1 向量 [δθ; δx; δy]
%
% 注意：
%   - 使用“左不变”定义，意味着误差在机体坐标系中解释
%   - 若使用右不变误差，应计算 log(X_hat * X⁻¹)

%% 1. 将真实状态 state 转换为 SE(2) 齐次变换矩阵
% 构造 3×3 的齐次变换矩阵：
%   chi = [ Rot   p  ]
%         [  0    1  ]
chi = [state.Rot, state.p;
       0,         0, 1];

%% 2. 将估计状态 hat_state 转换为 SE(2) 齐次变换矩阵
hat_chi = [hat_state.Rot, hat_state.p;
           0,              0, 1];

%% 3. 计算相对变换：Δχ = X⁻¹ * X_hat
% 即：从真实状态到估计状态的“相对位姿”
% 使用 se2_inv 函数计算 chi 的逆（高效实现，避免 full matrix inverse）
%   se2_inv(chi) = [ R', -R'*p;
%                    0,    1   ]
% 然后左作用于 hat_chi
relative_chi = se2_inv(chi) * hat_chi;

%% 4. 对相对变换进行李代数对数映射
% se2_log: SE(2) → se(2)，返回 3×1 向量 [δθ; δx; δy]
xi = se2_log(relative_chi);

% 函数结束
end