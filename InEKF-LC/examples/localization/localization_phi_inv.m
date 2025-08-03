function [xi] = localization_phi_inv(state, hat_state)
% LOCALIZATION_PHI_INV  计算欧氏误差（逆重牵引）
%
% 语法:
%   xi = localization_phi_inv(state, hat_state)
%
% 描述:
%   计算从真实状态到估计状态的欧氏误差：
%     - 方向误差：Δθ = log(R_hat * R^T)
%     - 位置误差：Δp = p_hat - p
%   适用于标准 EKF 的误差评估。
%
% 输入:
%   state     - 真实状态（.Rot, .p）
%   hat_state - 估计状态（.Rot, .p）
%
% 输出:
%   xi        - 误差向量 [δθ; δx; δy]

% 方向误差：相对旋转的对数映射（SO(2) 流形上测地线距离）
xi = [so2_log(hat_state.Rot * state.Rot');
      hat_state.p - state.p];
end