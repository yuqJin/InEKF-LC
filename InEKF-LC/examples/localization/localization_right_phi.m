function [new_state] = localization_right_phi(state, xi)
% LOCALIZATION_RIGHT_PHI  右不变状态更新（右作用重牵引）
%
% 语法:
%   new_state = localization_right_phi(state, xi)
%
% 描述:
%   实现 SE(2) 上的右不变更新：X ← exp(ξ) * X
%   即：在世界坐标系下施加一个小变换，适用于右不变误差框架。
%
% 输入:
%   state  - 当前状态（.Rot: 旋转矩阵, .p: 位置）
%   xi     - 李代数向量 [δθ; δx; δy]，表示世界坐标系下的小变换
%
% 输出:
%   new_state - 更新后的状态

% 计算指数映射：χ = exp(ξ) ∈ SE(2)
chi = se2_exp(xi);

% 更新方向：左乘小旋转（世界坐标系下旋转）
new_state.Rot = chi(1:2, 1:2) * state.Rot;

% 更新位置：p ← t + R * p_old（世界坐标系下的刚体变换）
new_state.p = chi(1:2, 3) + chi(1:2, 1:2) * state.p;
end