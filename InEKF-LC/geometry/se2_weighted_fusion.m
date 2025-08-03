function [fused_state, fused_P] = se2_weighted_fusion(states, Ps, weights)
    % 输入校验
    R = length(states);
    assert(abs(sum(weights) - 1) < 1e-6, '权重未归一化!');
    
    % 初始化参考状态（选择权重最大的状态）
    [~, ref_idx] = max(weights);
    ref_chi = [states(ref_idx).Rot, states(ref_idx).p; 0 0 1];
    
    % 迭代求解加权平均（通常3-5次迭代足够）
    for iter = 1:5
        sum_xi = zeros(3,1);
        for j = 1:R
            chi_j = [states(j).Rot, states(j).p; 0 0 1];
            xi_j = se2_log(ref_chi \ chi_j); % 右不变误差
            sum_xi = sum_xi + weights(j) * xi_j;
        end
        
        % 更新参考状态
        ref_chi = ref_chi * se2_exp(sum_xi);
    end
    
    % 提取融合后的状态
    fused_state = struct('Rot', ref_chi(1:2,1:2), 'p', ref_chi(1:2,3));
    
    % 协方差融合（在李代数空间）
    fused_P = zeros(3);
    for j = 1:R
        chi_j = [states(j).Rot, states(j).p; 0 0 1];
        xi_j = se2_log(ref_chi \ chi_j); % 相对于融合状态的误差
        fused_P = fused_P + weights(j) * (Ps(:,:,j) + xi_j * xi_j');
    end
end