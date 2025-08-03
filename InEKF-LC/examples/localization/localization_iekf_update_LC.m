function [up_state, up_P] = localization_iekf_update_LC(state, P, y, R, omega, dt)
% LOCALIZATION_IEKF_UPDATE_LC  带线性约束（Linear Constraint, LC）的 InEKF 更新
%
% 语法:
%   [up_state, up_P] = localization_iekf_update_LC(state, P, y, R, omega, dt)
%
% 描述:
%   本函数实现一种改进型不变扩展卡尔曼滤波（InEKF）的测量更新步骤，
%   引入了“线性约束”（Linear Constraint, LC）机制，
%
%   LC 的核心思想是：基于控制输入 omega 和时间步 dt，预测当前位置的期望变化 b_k，
%   并将该先验信息融入卡尔曼增益设计中，从而构造一个更具鲁棒性的更新过程。
%
%   该方法不改变状态更新的几何结构（仍使用左不变误差），但通过修正增益 L_k
%   和协方差更新项，提高了滤波器的一致性与收敛速度。
%
% 输入参数:
%   state   - 当前预测状态（结构体），包含：
%             .Rot: 2x2 旋转矩阵（方向，属于 SO(2)）
%             .p:   2x1 位置向量 [x; y]
%   P       - 当前预测协方差矩阵（3x3），表示状态不确定性
%   y       - 实际观测值（2x1 向量），如 GPS 测量 [x_gps; y_gps]
%   R       - 观测噪声协方差矩阵（2x2），通常为 diag(gps_noise_std^2)
%   omega   - 当前控制输入（结构体），包含：
%             .v:     前向线速度（m/s）
%             .gyro:  角速度（rad/s）
%   dt      - 时间步长（秒）
%
% 输出参数:
%   up_state - 更新后的状态（结构体）
%   up_P     - 更新后的协方差矩阵（3x3）
%

%% 1. 预测本次测量间隔内的相对运动（用于构建线性约束）

% 计算由角速度引起的姿态变化（小旋转）
Delta_Rot = so2_exp(omega.gyro * dt);

% 计算在机体坐标系中前向运动产生的位移，并通过 Delta_Rot 旋转到世界坐标系
Delta_p = Delta_Rot * (omega.v) * dt;

% 构造线性约束向量 b_k ∈ ℝ³，表示“若状态未变，位置应移动多少”
% 此处 b_k = [0; Delta_p]，即：
%   - 第1个元素为0：方向无内部偏移（仅由 gyro 积分）
%   - 第2-3个元素为预测的位置增量
b_k = [0; Delta_p];  % 3×1 向量 [δθ; δx; δy]，其中 δθ=0

%% 2. 构造观测模型的雅可比矩阵 H（基于左不变误差框架）

% 在 InEKF 框架下，观测函数 h(state)=p 的线性化应考虑当前姿态的影响
% 正确形式为：
%   H = Rot * [0_{2×1}, I_{2×2}]
% 表示位置观测受当前方向旋转的影响
H = state.Rot * [zeros(2, 1), eye(2)];

%% 3. 计算测量不确定性（创新协方差矩阵）

S = H * P * H' + R;  % 2x2 矩阵，表示预测观测的不确定性

%% 4. 构建线性约束影响项 G_k

% G_k = H * b_k
% 表示：由于系统运动，即使状态准确，测量也应预期发生变化 G_k
% 该项用于后续增益修正，实现“运动补偿型更新”
G_k = H * b_k;

%% 5. 计算修正型卡尔曼增益 L_k（LC 的核心）

% 标准卡尔曼增益
K = P * H' / S;  % 等价于 P*H'*inv(S)

% 定义增益修正方向：Gamma_k = b_k - K*G_k
Gamma_k = b_k - K * G_k;

% 计算标量权重 Sigma：衡量运动信息的相对置信度
Sigma = G_k' * inv(S) * G_k;  % 标量，越大表示运动信息越显著

% 构造最终的修正增益 L_k：
%   L_k = K + Gamma_k * (inv(Sigma) * G_k' * inv(S))
%
% 物理意义：
%   - 利用运动先验 b_k 调整增益，使滤波器对方向误差更敏感
%   - 在低激励运动中仍能有效修正航向偏差
L_k = K + Gamma_k * inv(Sigma) * G_k' * inv(S);

%% 6. 计算创新量并生成状态修正量 xi

% 观测残差（innovation）
innovation_residual = y - localization_h(state);

% 使用修正增益 L_k 映射残差为不变误差空间中的修正量
xi = L_k * innovation_residual;

%% 7. 使用左不变更新函数 phi 更新状态（保持几何一致性）

% 实现：X_hat ← X * exp(ξ)
% 即：
%   Rot ← Rot * exp(δθ)
%   p   ← p + Rot * [δx; δy]
up_state = localization_left_phi(state, xi);

%% 8. 更新协方差矩阵（包含 LC 修正项）

% 主项：标准 Joseph 形式更新
up_P = (eye(3) - K * H) * P;

% 附加项：补偿由于引入运动先验带来的协方差修正
% 保证整体估计的统计一致性
up_P = up_P + Gamma_k * inv(Sigma) * Gamma_k';

% 函数结束
end