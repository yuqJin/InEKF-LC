function [xi] = localization_right_phi_inv(state, hat_state)
% LOCALIZATION_RIGHT_PHI_INV  计算右不变误差（逆重牵引）
%
% 语法:
%   xi = localization_right_phi_inv(state, hat_state)
%
% 描述:
%   计算右不变误差：ξ = log(X_hat * X⁻¹)
%   表示从真实状态到估计状态的变换在**世界坐标系**下的李代数表示。
%   与 localization_right_phi 互为逆操作。
%
% 输入:
%   state     - 真实状态（.Rot, .p）
%   hat_state - 估计状态（.Rot, .p）
%
% 输出:
%   xi        - 右不变误差向量 [δθ; δx; δy]

% 构造 SE(2) 齐次变换矩阵
chi     = [state.Rot,     state.p;     0, 0, 1];
hat_chi = [hat_state.Rot, hat_state.p; 0, 0, 1];

% 计算相对变换：Δχ = X_hat * X⁻¹，再取对数映射
xi = se2_log(hat_chi * se2_inv(chi));
end