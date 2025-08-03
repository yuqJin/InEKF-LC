function [new_state] = localization_left_phi(state, xi)
% LOCALIZATION_LEFT_PHI  SE(2) 上的左不变状态更新（左作用重牵引）
%
% 语法:
%   new_state = localization_left_phi(state, xi)
%
% 描述:
%   本函数实现机器人位姿（位置 + 方向）在 SE(2) 李群上的左不变更新，
%   即通过李代数元素 xi 对当前状态进行修正，保持旋转矩阵的正交性与群结构一致性。
%
%   在不变扩展卡尔曼滤波（InEKF）中，状态误差定义为左不变误差：
%       ξ = log( X⁻¹ * X_hat ) ∈ se(2)
%   因此状态更新应采用左作用方式：
%       X_hat ← X * exp(ξ)
%   本函数正是实现这一更新规则，常用于 InEKF 和 InEKF-LC 的测量更新步骤。
%
%   与直接加法（如 theta = theta + dtheta）相比，该方法具有：
%     - 几何一致性（保持 SO(2) 和 SE(2) 结构）
%     - 数值稳定性（避免归一化问题）
%     - 更优的滤波器一致性表现
%
% 输入参数:
%   state   - 当前状态（结构体），包含：
%             .Rot: 2x2 旋转矩阵（方向，属于 SO(2)）
%             .p:   2x1 位置向量 [x; y]
%   xi      - 修正量（3×1 向量），表示左不变误差（李代数 se(2) 元素）
%             格式为：[δθ; δx; δy]
%             其中：
%               δθ: 航向角小旋转（弧度）
%               δx, δy: 在机体坐标系下的位置修正
%
% 输出参数:
%   new_state - 更新后的状态（结构体），满足：
%               new_state.Rot ∈ SO(2)
%               new_state.p ∈ ℝ²
%
% 数学原理:
%   1. 将 xi ∈ ℝ³ 映射到 SE(2) 李代数对应的变换矩阵：
%        chi = exp([Ω, v; 0, 0]) ∈ SE(2)
%      其中 Ω = so2_hat(δθ), v = [δx; δy]
%   2. 左乘当前状态：X ← X * chi
%      即：
%        R ← R * chi_R
%        p ← p + R * chi_t
%
% 注意：
%   - 使用“左不变”更新对应左群作用（left group action）
%   - 若使用右不变误差，应改为右乘：X ← exp(ξ) * X

%% 1. 将李代数向量 xi 映射为 SE(2) 变换矩阵（指数映射）
%   xi = [δθ; δx; δy] → chi ∈ SE(2)（3×3 齐次变换矩阵）
%
%   chi 形式如下：
%       [ cos(δθ)  -sin(δθ)   δx ]
%       [ sin(δθ)   cos(δθ)   δy ]
%       [    0         0       1  ]
%
%   se2_exp 函数负责实现该映射（假设已定义）
chi = se2_exp(xi);

%% 2. 更新方向（旋转矩阵）—— 左乘小旋转
% 新的方向 = 原方向 × 由 δθ 生成的小旋转矩阵
% 即：R ← R * exp(δθ)
new_state.Rot = state.Rot * chi(1:2, 1:2);

%% 3. 更新位置（考虑机体坐标系下的修正）
% 位置修正 [δx; δy] 是在“机体坐标系”中定义的
% 需通过当前旋转矩阵 R 变换到“世界坐标系”
% 然后加到原位置上：p ← p + R * [δx; δy]
new_state.p = state.p + state.Rot * chi(1:2, 3);

% 函数结束
end