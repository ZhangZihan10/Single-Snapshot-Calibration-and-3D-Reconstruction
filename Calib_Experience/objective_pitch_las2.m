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

function obj = objective_pitch_las2(pitch)
global ocam_model;
global Cent;
global Cent1;
global cam_pitch_opt;
global cam_roll_opt;
global cam_yaw_opt;

% 映射点
[x, y] = mapping_points(Cent, cam_roll_opt, cam_pitch_opt, cam_yaw_opt, 0, pitch, 1, ocam_model);
[x1, y1] = mapping_points(Cent1, cam_roll_opt, cam_pitch_opt, cam_yaw_opt, 0, pitch, 1, ocam_model);

% 数据预处理：排序、去重
[x, y] = sort_and_filter_points(x, y);
[x1, y1] = sort_and_filter_points(x1, y1);

% 鲁棒斜率估计
k1 = estimate_slope_robust_inline(x, y);
k2 = estimate_slope_robust_inline(x1, y1);

% 计算目标值
obj = abs(k1 - k2);


% --- 内联辅助函数1：数据排序和滤波 ---
    function [x_out, y_out] = sort_and_filter_points(x_in, y_in)
        % 1. 按x值排序
        [x_sorted, sort_idx] = sort(x_in);
        y_sorted = y_in(sort_idx);
        
        % 2. 移除重复或过近的点
        min_distance = 1e-3;
        keep_idx = true(size(x_sorted));
        for i = 2:length(x_sorted)
            if abs(x_sorted(i) - x_sorted(i-1)) < min_distance
                keep_idx(i) = false;
            end
        end
        
        x_filtered = x_sorted(keep_idx);
        y_filtered = y_sorted(keep_idx);
        
        % 3. 确保至少3个点
        if length(x_filtered) < 3
            % 如果太少，均匀采样
            n_points = min(5, length(x_sorted));
            idx = round(linspace(1, length(x_sorted), n_points));
            x_out = x_sorted(idx);
            y_out = y_sorted(idx);
        else
            x_out = x_filtered;
            y_out = y_filtered;
        end
    end

% --- 内联辅助函数2：鲁棒斜率估计 ---
    function slope = estimate_slope_robust_inline(x_vals, y_vals)
        if length(x_vals) < 2
            slope = 0;
            return;
        end
        
        % 使用最小二乘法
        x_mean = mean(x_vals);
        y_mean = mean(y_vals);
        
        numerator = 0;
        denominator = 0;
        
        for i = 1:length(x_vals)
            numerator = numerator + (x_vals(i) - x_mean) * (y_vals(i) - y_mean);
            denominator = denominator + (x_vals(i) - x_mean)^2;
        end
        
        if abs(denominator) < 1e-10
            slope = 0;
        else
            slope = numerator / denominator;
        end
        
        % 添加稳健性检查
        if abs(slope) > 1000
            slope = 0; % 防止异常斜率
        end
    end
end