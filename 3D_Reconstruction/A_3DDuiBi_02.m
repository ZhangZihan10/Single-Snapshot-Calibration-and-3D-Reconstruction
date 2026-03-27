%% 虚拟三维重构评价系统 (基于多障碍板真值) 计算每个障碍板 相机无旋转，高度500
%% 相机无旋转，高度500
clear; clc; close all;

%% 1. 参数设置：四个障碍板角点精确坐标 (单位: 米)
% 保留用户提供的原始角点数据
% 障碍板 1 前
obs3_corners = [
    -2.6, 2.8,  0; 
    0.55,  2.8,  0;
     0.55,  2.8,  3.01;  
     -2.6, 2.8,  3.01
     ];
% 障碍板 2  左
obs4_corners = [ 
     -3.16, -3.5,   0;  
    -3.16, -0.41,  0;
    -3.16, -0.41,  3;  
    -3.16, -3.5,   3
   ]; 
% 障碍板 3 后
obs2_corners = [
    -2.6, -5.07,  0;  
     0.6, -5.07,  0;
     0.6, -5.07,  2.9;
     -2.6, -5.07,  2.9  
     ];
% 障碍板 4  右
obs1_corners = [ 
     2.3, -1.4,  0; 
     2.3,  0.5, 0;
     2.3,  0.5, 2.7;  
     2.3, -1.4,  2.7 
     ];

% 将所有障碍板放入一个集合
all_obstacles = {obs1_corners, obs2_corners, obs3_corners, obs4_corners};
% 方便输出报告时的命名 (按照上面的集合索引排列)
obstacle_names = {'obs1_corners (右)', 'obs2_corners (后)', 'obs3_corners (前)', 'obs4_corners (左)'};

%% 2. 构建多目标真值模型 (Multi-Object GT Generation) 与 标签追踪
sampling_res = 0.02; % 2cm 采样精度
ptCloud_GT_locs = [];
labels_GT = [];

for i = 1:length(all_obstacles)
    corners = all_obstacles{i};
    if isempty(corners), continue; end
    
    % 计算该面板的局部向量
    v1 = corners(2,:) - corners(1,:);
    v2 = corners(4,:) - corners(1,:);
    
    % 在面板表面均匀采样
    [d1, d2] = meshgrid(0:sampling_res:1, 0:sampling_res:1);
    grid_points = corners(1,:) + d1(:)*v1 + d2(:)*v2;
    
    % 记录三维点位置
    ptCloud_GT_locs = [ptCloud_GT_locs; grid_points];
    % 记录这些点对应的障碍板序号，用于后续分离重构点云
    labels_GT = [labels_GT; repmat(i, size(grid_points, 1), 1)];
end

% 直接基于坐标数组创建包含所有障碍板的整体真值点云
%ptCloud_GT = pointCloud(ptCloud_GT_locs);

% 为所有点生成绿色 RGB 数据 [R, G, B]
% [0, 1, 0] 代表纯绿色，我们将其复制到每一个点
numPoints = size(ptCloud_GT_locs, 1);
greenColors = repmat([0, 1, 0], numPoints, 1);
% 创建带有颜色信息的点云对象
ptCloud_GT = pointCloud(ptCloud_GT_locs, 'Color', greenColors);

% 开启绘图窗口
figure('Name', '三维重构真值点云预览', 'Color', [1 1 1]);
pcshow(ptCloud_GT, 'MarkerSize', 20); % 绘制点云，MarkerSize 控制点的大小

% 图表美化
title('多障碍板真值模型 (Ground Truth)', 'FontSize', 14);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on;
axis equal; % 保持比例一致，防止变形
view(3);    % 设置为标准 3D 视角

% (可选) 为每个障碍板添加文字标注
%hold on;
%for i = 1:length(all_obstacles)
%    center_pos = mean(all_obstacles{i}); % 计算中心位置放置标签
%    text(center_pos(1), center_pos(2), center_pos(3)+0.2, ...
%        obstacle_names{i}, 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
%end
%hold off;

%% 3. 自动导入、识别并生成【3D 密集云图】
fig_file = 'image1_test.fig'; 
if ~exist(fig_file, 'file')
    error('未找到文件 %s', fig_file);
end
temp_fig = openfig(fig_file, 'invisible');
% 寻找所有图形对象
target_objs = findobj(temp_fig, 'Type', 'surface');
if isempty(target_objs), target_objs = findobj(temp_fig, 'Type', 'scatter'); end

% 初始化变量
X_final = []; 
Y_final = []; 
Z_final = [];
grid_res = 0.02; % 采样间距 2cm

if ~isempty(target_objs)
    for k = 1:length(target_objs)
        % 提取原始数据并缩放
        x_raw = get(target_objs(k), 'XData') / 1000;
        y_raw = get(target_objs(k), 'YData') / 1000;
        z_raw = get(target_objs(k), 'ZData') / 1000;
        
        % 提取物理边界以判断面板朝向
        x_min = min(x_raw(:)); x_max = max(x_raw(:));
        y_min = min(y_raw(:)); y_max = max(y_raw(:));
        z_min = min(z_raw(:)); z_max = max(z_raw(:));
        
        % 根据 Z 轴差值判断是否需要垂直织网
        if (z_max - z_min) > 0.1
            % 判断面板在 XY 平面的走向（是平行于 X 还是平行于 Y）
            if abs(x_max - x_min) > abs(y_max - y_min)
                [Xg, Zg] = meshgrid(x_min:grid_res:x_max, z_min:grid_res:z_max);
                Yg = ones(size(Xg)) * mean(y_raw(:));
            else
                [Yg, Zg] = meshgrid(y_min:grid_res:y_max, z_min:grid_res:z_max);
                Xg = ones(size(Yg)) * mean(x_raw(:));
            end
            X_final = [X_final; Xg(:)];
            Y_final = [Y_final; Yg(:)];
            Z_final = [Z_final; Zg(:)];
        else
            X_final = [X_final; x_raw(:)];
            Y_final = [Y_final; y_raw(:)];
            Z_final = [Z_final; z_raw(:)];
        end
    end
else
    close(temp_fig); 
    error('未找到 3D 数据，请检查 .fig 文件内容。');
end
close(temp_fig);

valid = ~isnan(X_final) & ~isnan(Y_final) & ~isnan(Z_final);
ptCloud_Recon = pointCloud([X_final(valid), Y_final(valid), Z_final(valid)]);

% 可视化初始点云图
figure('Color', 'w', 'Name', '3D 重构云图展示');
pcshow(ptCloud_Recon, 'MarkerSize', 15);
colormap(jet); 
cb = colorbar;
ylabel(cb, '高度/Z轴位置 (m)');
title('重构出的障碍板 3D 密集云图 (垂直面生成)');
view(3); axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

%% 4. 坐标系自动对齐 (Registration) 
fprintf('正在执行 ICP 坐标系精细对齐...\n');
[tform, ptCloud_Recon_Aligned] = pcregistericp(ptCloud_Recon, ptCloud_GT);

%% 5. 定量指标计算与分块统计 (Quantitative Metrics)
% 使用最近邻搜索(knnsearch)，找寻每个重构点离哪一个真值点最近
[idx_closest, dists] = knnsearch(ptCloud_GT_locs, ptCloud_Recon_Aligned.Location);

% === 计算全局误差 ===
MAE_total   = mean(dists);            
RMSE_total  = sqrt(mean(dists.^2));   
MedAE_total = median(dists);          

fprintf('\n--- 总体重构评价计算结果 ---\n');
fprintf('总体 MAE:   %.4f m (%.2f mm)\n', MAE_total, MAE_total * 1000);
fprintf('总体 RMSE:  %.4f m (%.2f mm)\n', RMSE_total, RMSE_total * 1000);
fprintf('总体 MedAE: %.4f m\n', MedAE_total);

% === 利用标签，识别每个重构点属于哪个具体的障碍板 ===
recon_labels = labels_GT(idx_closest);

% 分配不同颜色矩阵，用于第6部分可视化渲染各个障碍板
color_map = lines(length(all_obstacles));
recon_colors = zeros(size(ptCloud_Recon_Aligned.Location, 1), 3);

fprintf('\n--- 各个障碍板分别计算结果 ---\n');
for i = 1:length(all_obstacles)
    if isempty(all_obstacles{i}), continue; end
    
    % 筛选出被划分为该障碍板的所有重构点的距离和索引
    idx_i = (recon_labels == i);
    dists_i = dists(idx_i);
    
    if isempty(dists_i)
        fprintf('%s: 未匹配到相关的重构点。\n', obstacle_names{i});
        continue;
    end
    
    % 分别计算 MAE, RMSE, MedAE
    MAE_i   = mean(dists_i);
    RMSE_i  = sqrt(mean(dists_i.^2));
    MedAE_i = median(dists_i);
    
    fprintf('%s (包含 %d 个扫描点):\n', obstacle_names{i}, sum(idx_i));
    fprintf('  MAE:   %.4f m (%.2f mm)\n', MAE_i, MAE_i * 1000);
    fprintf('  RMSE:  %.4f m (%.2f mm)\n', RMSE_i, RMSE_i * 1000);
    fprintf('  MedAE: %.4f m\n\n', MedAE_i);
    
    % 为归属该障碍板的重构点分配固定颜色，便于展示
    recon_colors(idx_i, :) = repmat(color_map(i, :), sum(idx_i), 1);
end

%% 6. 结果可视化报告 (多色区分显示与自动图例)
figure('Color', 'w', 'Name', '3D 重建精度对比可视化 (分类着色)');
% --- 1. 绘制真值模型 (全部设置为绿色 [0 1 0]) ---
pcshow(ptCloud_GT.Location, [0 1 0], 'VerticalAxis', 'Z', 'VerticalAxisDir', 'Down', 'MarkerSize', 8); 
hold on;

% --- 2. 绘制对齐后的重构模型 (按障碍板归属赋予不同颜色) ---
ptCloud_ColoredRecon = pointCloud(ptCloud_Recon_Aligned.Location, 'Color', uint8(recon_colors * 255));
pcshow(ptCloud_ColoredRecon, 'MarkerSize', 15);

% --- 3. 图形修饰与图例生成 ---
title(['3D 重建 vs 真值模型 (总体 MAE: ', num2str(MAE_total*1000, '%.2f'), ' mm)'], 'FontSize', 12);

% 构建动态图例以适配颜色映射
h_gt = scatter3(NaN, NaN, NaN, 50, [0 1 0], 'filled'); % 隐藏的散点辅助建立图例
legend_handles = h_gt;
legend_labels = {'真值模型 (GT 全局参考)'};

for i = 1:length(all_obstacles)
    if any(recon_labels == i)
        h_tmp = scatter3(NaN, NaN, NaN, 50, color_map(i,:), 'filled');
        legend_handles = [legend_handles, h_tmp];
        legend_labels{end+1} = [obstacle_names{i} ' 重构点'];
    end
end

legend(legend_handles, legend_labels, 'TextColor', 'black', 'Location', 'northeast');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(3);              
axis equal;           
grid on;              
set(gca, 'Color', [0.1 0.1 0.1]);