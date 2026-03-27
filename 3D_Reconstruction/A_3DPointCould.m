%% 虚拟三维重构评价系统 (基于多障碍板真值)
clear; clc; close all;


%% 3. 自动导入、识别并生成【3D 密集云图】
fig_file = 'test5_O.fig'; 
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
grid_res = 0.005; % 采样间距 2cm

% 3. 遍历识别到的对象并基于角点构建任意角度平面
if ~isempty(target_objs)
    for k = 1:length(target_objs)
        % 提取原始角点数据并缩放
        x_raw = get(target_objs(k), 'XData') / 100;
        y_raw = get(target_objs(k), 'YData') / 100;
        z_raw = get(target_objs(k), 'ZData') / 100;
        
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

