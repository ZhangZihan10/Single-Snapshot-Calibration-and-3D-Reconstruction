%% 虚拟三维重构评价系统 (基于多障碍板真值)  有角度
clear; clc; close all;

%% 1. 参数设置：四个障碍板角点精确坐标 (单位: 米)
% 保留用户提供的原始角点数据，不作任何修改

% 障碍板 1
obs3_corners = [
     
    -2.4, 2.1,  0; 
    0.42,  3.4,  0;
     0.42,  3.4,  2.9;  
     -2.4, 2.1,  2.9
     ];

% 障碍板 2
obs4_corners = [ 
      
     -2.3, -3.4,   0;  
    -3.9, -0.6,  0;
    -3.9, -0.6,  2.9;  
    -2.3, -3.4,   2.9
   ]; 

% 障碍板 3
obs2_corners = [
      
    0.37, -5.7,  0;  
     -2.3, -4.2,  0
     -2.3, -4.2,  2.9;
     0.37, -5.7,  2.9  
     ];

% 障碍板 4
obs1_corners = [ 
      
     2.5, 0.45,  0; 
     2.08,  -1.3, 0;
     2.08,  -1.3, 2.7;  
     2.5, 0.45,  2.7 
     
     ];

% 将所有障碍板放入一个集合
all_obstacles = {obs1_corners, obs2_corners, obs3_corners, obs4_corners};

%% 2. 构建多目标真值模型 (Multi-Object GT Generation)
sampling_res = 0.05; % 1cm 采样精度
ptCloud_GT = pointCloud(zeros(0,3)); % 统一变量名为 ptCloud_GT

for i = 1:length(all_obstacles)
    corners = all_obstacles{i};
    if isempty(corners), continue; end
    
    % 计算该面板的局部向量
    v1 = corners(2,:) - corners(1,:);
    v2 = corners(4,:) - corners(1,:);
    
    % 在面板表面均匀采样
    [d1, d2] = meshgrid(0:sampling_res:1, 0:sampling_res:1);
    grid_points = corners(1,:) + d1(:)*v1 + d2(:)*v2;
    
    % 合并到总真值模型
    ptCloud_GT = pcmerge(ptCloud_GT, pointCloud(grid_points), 0.001);
end


%% 3. 自动导入、识别并生成【3D 密集云图】
fig_file = 'image3_test.fig'; 
if ~exist(fig_file, 'file')
    error('未找到文件 %s', fig_file);
end

temp_fig = openfig(fig_file, 'invisible');
% 寻找所有图形对象
target_objs = findobj(temp_fig, 'Type', 'surface');
if isempty(target_objs), target_objs = findobj(temp_fig, 'Type', 'scatter'); end

% --- 在进入循环提取前，必须先初始化变量 ---
X_final = []; 
Y_final = []; 
Z_final = [];
grid_res = 0.05; % 采样间距 2cm

% 3. 遍历识别到的对象并基于角点构建任意角度平面
if ~isempty(target_objs)
    for k = 1:length(target_objs)
        % 提取原始角点数据并缩放
        x_raw = get(target_objs(k), 'XData') / 1000;
        y_raw = get(target_objs(k), 'YData') / 1000;
        z_raw = get(target_objs(k), 'ZData') / 1000;
        
        % 将 2x2 或 1x4 的角点矩阵转换为 4x3 的点列表
        pts = [x_raw(:), y_raw(:), z_raw(:)];
        
        % 只有当点数足以构成面（通常为 4 个角点）时进行面构建
        if size(pts, 1) >= 4
            % 假设角点顺序为：1:左上, 2:左下, 3:右上, 4:右下
            % 定义两个相邻的边缘向量（基向量）
            v_vertical   = pts(2,:) - pts(1,:); % 垂直边 (1 -> 2)
            v_horizontal = pts(3,:) - pts(1,:); % 水平边 (1 -> 3)
            
            % 计算面板的物理尺寸以确定采样点数
            len_v = norm(v_vertical);
            len_h = norm(v_horizontal);
            
            % 生成参数化坐标网格 [0, 1]
            [s, t] = meshgrid(0:(grid_res/len_h):1, 0:(grid_res/len_v):1);
            
            % 使用双线性合成生成平面上的所有点
            % 公式：P(s,t) = P1 + s*v_horizontal + t*v_vertical
            X_grid = pts(1,1) + s*v_horizontal(1) + t*v_vertical(1);
            Y_grid = pts(1,2) + s*v_horizontal(2) + t*v_vertical(2);
            Z_grid = pts(1,3) + s*v_horizontal(3) + t*v_vertical(3);
            
            X_final = [X_final; X_grid(:)];
            Y_final = [Y_final; Y_grid(:)];
            Z_final = [Z_final; Z_grid(:)];
        else
            % 如果点数不足（如离群点），直接拼接
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

% --- 此时运行到第 119 行，变量已存在 ---
valid = ~isnan(X_final) & ~isnan(Y_final) & ~isnan(Z_final);
ptCloud_Recon = pointCloud([X_final(valid), Y_final(valid), Z_final(valid)]);

% 绘制点云图
figure('Color', 'w', 'Name', '3D 重构云图展示');
pcshow(ptCloud_Recon, 'MarkerSize', 15);
colormap(jet); 
cb = colorbar;
ylabel(cb, '高度/Z轴位置 (m)');
title('重构出的障碍板 3D 密集云图 (垂直面生成)');

% 强制切换视角
view(3); 
axis equal; 
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

%% 4. 坐标系自动对齐 (Registration) 
% 由于手动测量坐标系与相机坐标系通常不重合，执行 ICP 精细对齐
fprintf('正在执行 ICP 坐标对齐...\n');
[tform, ptCloud_Recon_Aligned] = pcregistericp(ptCloud_Recon, ptCloud_GT);

%% 5. 定量指标计算 (Quantitative Metrics) [cite: 365, 398]
% 使用最近邻搜索计算欧式距离误差 (RMSE, MAE 计算基础) [cite: 374]
[~, dists] = knnsearch(ptCloud_GT.Location, ptCloud_Recon_Aligned.Location);

% 计算核心评价指标 [cite: 398]
MAE   = mean(dists);            % 平均绝对误差
RMSE  = sqrt(mean(dists.^2));   % 均方根误差
MedAE = median(dists);          % 中值绝对误差


%% 6. 结果可视化与报告输出
fprintf('\n--- 基于 RESDEPTH 公式计算结果 ---\n');
%fprintf('点云总数 (N): %d\n', N);
fprintf('MAE (公式 1):  %.4f m (%.2f mm)\n', MAE, MAE * 1000);
fprintf('RMSE (公式 2): %.4f m (%.2f mm)\n', RMSE, RMSE * 1000);
fprintf('MedAE:        %.4f m\n', MedAE);


%% 6. 结果可视化报告 (多色区分显示)
figure('Color', 'w', 'Name', '3D 重建精度对比可视化');

% --- 1. 绘制真值模型 (设置为绿色 [0 1 0]) ---
% 将 MarkerSize 设小一点，使其看起来像一个连续的参考平面
pcshow(ptCloud_GT.Location, [0 1 0], 'VerticalAxis', 'Z', 'VerticalAxisDir', 'Down'); 
hold on;

% --- 2. 绘制对齐后的重构模型 (设置为橙红色 [1 0.4 0]) ---
% 增加 MarkerSize 以便观察重构点的分布细节
pcshow(ptCloud_Recon_Aligned.Location, [1 0.4 0], 'MarkerSize', 15);

% --- 3. 图形修饰 ---
% 动态生成包含 MAE 的标题
title(['3D 重建 vs 真值模型 (MAE: ', num2str(MAE*1000, '%.2f'), ' mm)'], 'FontSize', 12);

% 设置图例：确保颜色与标签对应
legend('真值模型 (Unity GT - Green)', '重构模型 (Reconstructed - Orange)', ...
       'TextColor', 'black', 'Location', 'northeast');

% 坐标轴与视角设置
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(3);              % 切换到 3D 视角
axis equal;           % 保持 1:1:1 的物理比例，防止障碍板被压扁
grid on;              % 显示网格
set(gca, 'Color', [0.1 0.1 0.1]); % 可选：将背景设为深色，使彩色点云更突出


