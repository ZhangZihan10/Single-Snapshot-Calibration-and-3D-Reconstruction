function [laser_mask, laser_regions] = filter_laser_lines(binary_img, min_length, max_width, min_aspect_ratio, solidity_threshold)
% 从二值图像中检测并保留激光线段
%
% 参数:
%   binary_img: 二值图像（包含激光线段和其他噪声）
%   min_length: 激光线段最小长度 (默认: 30)
%   max_width: 激光线段最大宽度 (默认: 10)
%   min_aspect_ratio: 最小长宽比 (默认: 3.0)
%   solidity_threshold: 实心度阈值 (默认: 0.8)
%
% 返回:
%   laser_mask: 只包含激光线段的二值图像
%   laser_regions: 检测到的激光线段信息结构体

% 设置默认参数
if nargin < 2
    min_length = 30;
end
if nargin < 3
    max_width = 10;
end
if nargin < 4
    min_aspect_ratio = 3.0;
end
if nargin < 5
    solidity_threshold = 0.8;
end

% 连通域分析
labeled = bwlabel(binary_img > 0);
stats = regionprops(labeled, 'BoundingBox', 'Area', 'Perimeter', 'Solidity', 'Orientation');

% 初始化激光掩码
laser_mask = false(size(binary_img));
laser_regions = struct();

region_count = 0;

for i = 1:length(stats)
    region = stats(i);
    
    % 获取边界框尺寸
    bbox = region.BoundingBox;
    width = bbox(3);   % 宽度
    height = bbox(4);  % 高度
    
    % 计算主要特征
    length_val = max(height, width);      % 主要尺寸
    short_side = min(height, width);      % 次要尺寸
    
    % 避免除以零
    if short_side > 0
        aspect_ratio = length_val / short_side;
    else
        aspect_ratio = 0;
    end
    
    area = region.Area;
    perimeter = region.Perimeter;
    solidity = region.Solidity;           % 实心度
    orientation = region.Orientation;     % 方向角
    
    % 激光线段的判断条件
    is_laser_line = (length_val >= min_length) && ...
                    (short_side <= max_width) && ...
                    (aspect_ratio >= min_aspect_ratio) && ...
                    (solidity >= solidity_threshold);
    
    if is_laser_line
        % 保留这个激光线段
        region_pixels = (labeled == i);
        laser_mask = laser_mask | region_pixels;
        
        region_count = region_count + 1;
        
        % 存储区域信息
        laser_regions(region_count).length = length_val;
        laser_regions(region_count).width = short_side;
        laser_regions(region_count).aspect_ratio = aspect_ratio;
        laser_regions(region_count).solidity = solidity;
        laser_regions(region_count).orientation = orientation;
        laser_regions(region_count).area = area;
        
        fprintf('检测到激光线段: 长度=%.1f, 宽度=%.1f, 长宽比=%.2f, 实心度=%.3f, 方向角=%.1f°\n', ...
                length_val, short_side, aspect_ratio, solidity, orientation);
    end
end

% 如果没有检测到任何区域，返回空结构体
if region_count == 0
    laser_regions = struct();
end

end