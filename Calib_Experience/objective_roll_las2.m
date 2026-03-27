%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%   Author: Ivan Kholodilin
%   email: kholodilin@bit.edu.cn
%   website: www.ilabit.org
%
%   Copyright (C) 2020 Ivan Kholodilin
%   
%   This program is free software; you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation; either version 2 of the License, or
%   (at your option) any later version.
%   
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%   
%   You should have received a copy of the GNU General Public License
%   along with this program; if not, write to the Free Software
%   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
%   USA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function obj = objective_roll_las2(roll)
global ocam_model;
global Cent;
global Cent1;
global cam_pitch_opt;
global cam_roll_opt;
global cam_yaw_opt;

% 映射点
[x, y] = mapping_points(Cent, cam_roll_opt, cam_pitch_opt, cam_yaw_opt, roll, 0, 1, ocam_model);
[x1, y1] = mapping_points(Cent1, cam_roll_opt, cam_pitch_opt, cam_yaw_opt, roll, 0, 1, ocam_model);

% 数据预处理
[x, y] = sort_and_filter_points(x, y);
[x1, y1] = sort_and_filter_points(x1, y1);

% 方法1：使用中位数（对异常值鲁棒）
median_y1 = median(y);
median_y2 = median(y1);
obj_median = abs(median_y1 - median_y2);

% 方法2：使用加权平均（中心点权重更高）
if length(x) > 3 && length(x1) > 3
    % 第一条线
    x_center = (max(x) + min(x)) / 2;
    x_std = (max(x) - min(x)) / 4;
    if x_std < eps, x_std = 1; end
    weights1 = exp(-((x - x_center) / x_std).^2);
    weighted_mean_y1 = sum(y .* weights1) / sum(weights1);
    
    % 第二条线
    x_center2 = (max(x1) + min(x1)) / 2;
    x_std2 = (max(x1) - min(x1)) / 4;
    if x_std2 < eps, x_std2 = 1; end
    weights2 = exp(-((x1 - x_center2) / x_std2).^2);
    weighted_mean_y2 = sum(y1 .* weights2) / sum(weights2);
    
    obj_weighted = abs(weighted_mean_y1 - weighted_mean_y2);
else
    obj_weighted = obj_median;
end

% 方法3：检查线的"水平性"（惩罚倾斜的线）
flatness_penalty = 0;
if length(x) >= 2 && length(x1) >= 2
    slope1 = estimate_simple_slope(x, y);
    slope2 = estimate_simple_slope(x1, y1);
    flatness_penalty = 0.1 * (abs(slope1) + abs(slope2));
end

% 组合目标函数
obj = 0.7 * min(obj_median, obj_weighted) + 0.3 * flatness_penalty;

% 添加小的正则化项防止过度优化
obj = obj + 0.0001 * (std(y)^2 + std(y1)^2);

% --- 内联辅助函数1：数据排序和滤波 ---
    function [x_out, y_out] = sort_and_filter_points(x_in, y_in)
        % 1. 按x值排序
        [x_sorted, sort_idx] = sort(x_in);
        y_sorted = y_in(sort_idx);
        
        % 2. 简单的异常值过滤（使用3σ原则）
        if length(x_sorted) >= 5
            y_mean = mean(y_sorted);
            y_std = std(y_sorted);
            valid_idx = (y_sorted >= y_mean - 3*y_std) & (y_sorted <= y_mean + 3*y_std);
            x_sorted = x_sorted(valid_idx);
            y_sorted = y_sorted(valid_idx);
        end
        
        % 3. 确保足够点数
        if length(x_sorted) < 2
            x_out = x_in;
            y_out = y_in;
        else
            x_out = x_sorted;
            y_out = y_sorted;
        end
    end

% --- 内联辅助函数2：简单斜率估计 ---
    function slope = estimate_simple_slope(x_vals, y_vals)
        if length(x_vals) < 2
            slope = 0;
            return;
        end
        
        % 使用首尾点估计斜率（简单但稳定）
        slope = (y_vals(end) - y_vals(1)) / (x_vals(end) - x_vals(1) + eps);
    end
end