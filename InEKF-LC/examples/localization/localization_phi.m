function [new_state] = localization_phi(state, xi)
% LOCALIZATION_PHI  状态更新（右扰动模型）
%
% 语法:
%   new_state = localization_phi(state, xi)
%
% 描述:
%   使用右扰动模型更新机器人位姿：
%     - 方向：通过 SO(2) 指数映射添加角度误差
%     - 位置：直接相加（欧氏空间更新）
%   该方法适用于标准 EKF 等基于欧氏误差的滤波器。
%
% 输入:
%   state - 当前状态（.Rot: 旋转矩阵, .p: 位置）
%   xi    - 误差向量 [δθ; δx; δy]（小量修正）
%
% 输出:
%   new_state - 更新后的状态

% 更新方向：当前姿态右乘小旋转
new_state.Rot = state.Rot * so2_exp(xi(1));

% 更新位置：直接在世界坐标系中加偏移
new_state.p = state.p + xi(2:3);
end