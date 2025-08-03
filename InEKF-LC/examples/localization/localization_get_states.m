function [Rots, ps] = localization_get_states(states)
% LOCALIZATION_GET_STATES  从状态结构体数组中提取方向和位置轨迹
%
% 语法:
%   [Rots, ps] = localization_get_states(states)
%
% 输入参数:
%   states - 状态轨迹（1×N 的结构体数组），每个元素包含：
%            .Rot: 2x2 旋转矩阵（方向，属于 SO(2)）
%            .p:   2x1 位置向量 [x; y]
%
% 输出参数:
%   Rots - 方向轨迹（元胞数组，1×N），每个元素是 2x2 旋转矩阵
%   ps   - 位置轨迹（N×2 矩阵），每行对应一个时刻的 [x, y] 坐标
%
% 功能说明：
%   该函数将结构体数组形式的状态轨迹转换为便于分析和绘图的格式：
%     - Rots 保留为元胞数组，方便逐个访问旋转矩阵（如用于方向误差计算）
%     - ps 转换为普通数值矩阵，便于直接用于 plot、rms、diff 等函数
%
% 应用场景：
%   常用于滤波结果的后处理，例如绘制轨迹、计算误差、生成动画等。

%% 将结构体数组转换为表格（struct2table）
% MATLAB 的 struct2table 函数将结构体数组转换为表格类型（table）
% 每个字段变为一列，每行对应一个时间步
state_table = struct2table(states);

%% 提取方向矩阵序列
% state_table.Rot 是一个 1×N 的元胞数组（cell array），每个单元包含一个 2x2 矩阵
% 直接赋值给 Rots，保持元胞数组格式，便于后续按索引访问
Rots = state_table.Rot;

%% 提取位置序列并转换为数值矩阵
% state_table.p 是一个 1×N 的元胞数组，每个单元包含一个 2×1 向量 [x; y]
%
% 步骤分解：
%   state_table.p      -> { [x1;y1], [x2;y2], ..., [xN;yN] }   (1×N cell)
%   state_table.p'     -> 转置为 N×1 cell
%   cell2mat(...)      -> 将 N×1 cell 拼接成 N×2 数值矩阵
%   '                  -> 再次转置，得到最终的 N×2 矩阵（每行一个 [x,y]）
ps = cell2mat(state_table.p')';

% 示例：
%   若 state_table.p = { [1;2], [3;4], [5;6] }
%   则 state_table.p' = { [1;2]; [3;4]; [5;6] }  (3×1 cell)
%   cell2mat(state_table.p') = [1 2; 3 4; 5 6]   (3×2 matrix)
%   最后转置：ps = [1 3 5; 2 4 6]' → [1 2; 3 4; 5 6] （N×2）

% 函数结束
end