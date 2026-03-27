%% 只进行3D重建（虚拟环境）
%% 2. 遍历所有图像并进行透视变换和3D建模
clear

load('fisheyeCameraParams.mat');
load('Omni_Calib_Results_Unity.mat');
ocam_model = calib_data.ocam_model;
intrinsics = cameraParams.Intrinsics;

% 激光参数
camX = 1.01; camY = 1.92; camZ = 3.012;
lasX = 0; lasY = 0;
CVsyst_rot = 0;
las_dist = 4950;
%CVsyst_x = 1800;
%CVsyst_y = 3400;
CVsyst_x = 0;
CVsyst_y = 0;

Cube_l = 450;  % 底边实际长度（单位 mm）
fov = 115;
f_max = 611.6;  % 等效焦距

% 获取图像列表
outputFolder = 'obstacle_boards3';
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

folderPath = outputFolder;
imageFiles = dir(fullfile(folderPath, '*.jpg'));

% 所有障碍板在统一坐标系下显示
figure(99); clf;  % 统一图窗，清空旧内容
ax = axes;        % 获取当前坐标轴句柄
hold(ax, 'on');   % 确保只 hold 一个轴
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
axis(ax, 'equal');
axis(ax, [0 25 0 25 0 25]);
grid(ax, 'on');
view(ax, 3);
title(ax, '所有障碍板贴图结果');

for i = 1:length(imageFiles)

    filename = fullfile(folderPath, imageFiles(i).name);
    testImage = imread(filename);
    testImage = imresize(testImage, [1920, 1920]);

    % 鱼眼矫正
    correctedImg = undistortFisheyeImage(testImage, intrinsics, 'OutputView', 'same', 'ScaleFactor', 0.4);
    % figure;imshow(correctedImg);
    % 激光识别 + 坐标映射
    img = LaserFind(testImage);
    %figure;imshow(img);
    %img = las_segm(testImage);%传统激光识别方法
    [x1, y1] = mapping(img, CVsyst_rot, CVsyst_x, CVsyst_y, camX, camY, camZ, lasX, lasY, las_dist, ocam_model);
    x1 = x1(2:end); y1 = y1(2:end);
    [xt1, yt1] = MappingRobot([x1', y1'], Cube_l);
    % 对 xt1 和 yt1 进行一次线性拟合
    %figure;scatter(xt1,yt1,5,'filled',"green");

  
    % 假设 xt1, yt1 是你的点列向量
    %pts = [xt1(:), yt1(:)];  % N×2矩阵  
    pts = [x1(:), y1(:)]; 
    % 1. 计算点集质心（均值）
    meanPt = mean(pts, 1);    
    % 2. 将点平移到质心（中心化）
    pts_centered = pts - meanPt;
    % 3. 对中心化后的点做奇异值分解（SVD）
    [~, ~, V] = svd(pts_centered, 0);   
    % 4. 方向向量（第一奇异向量对应主方向）
    dir_vector = V(:,1);  % 2×1向量，单位长度    
    % 5. 构造拟合直线的点参数方程：// 点：meanPt，方向：dir_vector
    % 参数t的范围，你自己调整以控制线段长度
    t = linspace(-10, 10, 100);     
    % 直线上点坐标
    x_line = meanPt(1) + t * dir_vector(1);
    y_line = meanPt(2) + t * dir_vector(2);
    
    % 方向向量与 y 轴的夹角（限制在 Y轴右侧，范围 0~180°）
    v_y = [0, 1];  % Y轴方向
    dir = dir_vector(:)';  % 行向量
    
    % 单位化
    v_y = v_y / norm(v_y);
    dir = dir / norm(dir);
    
    % 计算余弦夹角
    cos_theta = dot(v_y, dir);
    cos_theta = max(min(cos_theta, 1), -1);  % 避免浮点误差
    theta_rad = acos(cos_theta);
    theta_deg = rad2deg(theta_rad);
    
    % 如果方向在Y轴左侧（X分量 < 0），反转角度到右侧
    if dir(1) < 0
        theta_deg = 180 - theta_deg;
    end
    
    % 显示结果
    fprintf('线段与 Y 轴的夹角（右侧范围）：%.2f°\n', theta_deg);

    
    
    % 计算线段的长度
    % 初始化最大距离
    L = 0;
    p1 = [0, 0];
    p2 = [0, 0];
    
    % 将点集合组合为二维坐标点矩阵
    %points = [xt1(:), yt1(:)];  % 强制列向量确保维度一致
    points = [x1(:), y1(:)];
    N = size(points, 1);
    
    % 双重循环查找最远两点
    for i1 = 1:N
        for j = i1+1:N
            d = norm(points(i1,:) - points(j,:));  % 欧几里得距离
            if d > L
                L = d;
                p1 = points(i1,:);
                p2 = points(j,:);
            end
        end
    end
    fprintf('线段长度: %.2f\n', L);

    % 相机旋转
     % 根据图像序号设置姿态角
    switch i
        case 1
            roll = 0; pitch = 90; yaw = 90;
                if abs(theta_deg - 0) < 1 || abs(theta_deg - 180) < 1|| abs(theta_deg - 90) < 1
                    yaw2 = 0;
                elseif theta_deg>=90
                    yaw2 = 180-theta_deg+2;
                else
                    yaw2 = -(theta_deg+2);
                end
            R = eul2rotm(deg2rad([yaw, pitch,  yaw2]), 'ZYZ');
        case 2
            roll = -90; pitch = 0;
                if abs(theta_deg - 0) < 1 || abs(theta_deg - 180) < 1|| abs(theta_deg - 90) < 1
                    yaw = 0;
                elseif theta_deg<=90
                    yaw = theta_deg+2;
                else
                    yaw = 90-(theta_deg-2);
                end
            R = eul2rotm(deg2rad([roll,pitch,yaw]), 'XYZ');
        case 3
            roll = 90; pitch = 0;
                if abs(theta_deg - 0) < 1 || abs(theta_deg - 180) < 1 || abs(theta_deg - 90) < 1
                    yaw = 0;
                elseif theta_deg<=90
                    yaw = -theta_deg-2;
                else
                    yaw = theta_deg-2-90;
                end
            R = eul2rotm(deg2rad([roll,pitch,yaw]), 'XYZ');
        case 4
            roll = 0; pitch = -90; yaw = -90;
                if abs(theta_deg - 0) < 1 || abs(theta_deg - 180) < 1|| abs(theta_deg - 90) < 1
                    yaw2 = 0;
                elseif theta_deg<=90
                    yaw2 = -theta_deg-2;
                else
                    yaw2 = theta_deg-2-90;
                end
            R = eul2rotm(deg2rad([yaw, pitch,  yaw2]), 'ZYZ');
        otherwise, roll = 0; pitch = -90; yaw = -90;
    end
    theta_max1 = deg2rad(fov) / 2;
    grid_scale = tan(theta_max1) * 6;
    [u, v] = meshgrid(linspace(-grid_scale, grid_scale, 1920), linspace(-grid_scale, grid_scale, 1920));
    P = [u(:), ones(numel(u),1), v(:)]';
    P_rotated = R * P;
    theta = atan2(P_rotated(3,:), P_rotated(1,:));
    phi = atan2(vecnorm(P_rotated([1,3],:), 2, 1), P_rotated(2,:));

    I = (2 * phi .* cos(theta)) / deg2rad(fov);
    J = (2 * phi .* sin(theta)) / deg2rad(fov);
    S_inv = inv(intrinsics.StretchMatrix);
    I_corrected = S_inv(1,1)*I + S_inv(1,2)*J;
    J_corrected = S_inv(2,1)*I + S_inv(2,2)*J;
    x = I_corrected * f_max + intrinsics.DistortionCenter(1);
    y = J_corrected * f_max + intrinsics.DistortionCenter(2);

    % 双线性插值
    perspectiveImg = zeros(size(testImage), 'like', testImage);
    for c = 1:3
        perspectiveImg(:,:,c) = interp2(double(testImage(:,:,c)), reshape(x,1920,1920), reshape(y,1920,1920), 'linear', 0);
    end

    % 提取非黑像素区域
    mask = ~(perspectiveImg(:,:,1)<45 & perspectiveImg(:,:,2)<45 & perspectiveImg(:,:,3)<45);
    % figure;imshow(perspectiveImg);

    [row_idx, col_idx] = find(mask);
    if isempty(row_idx)
        continue;
    end
    h_pixel = max(row_idx) - min(row_idx);
    w_pixel = max(col_idx) - min(col_idx);
    H_real = L * (h_pixel / w_pixel);

    % 裁剪贴图
    img_crop = perspectiveImg(min(row_idx):max(row_idx), min(col_idx):max(col_idx), :);

    %figure;imshow(img_crop)
    
    switch i
        case 3
            if abs(theta_deg - 0) > 1 || abs(theta_deg - 180) > 1
                img_crop=imrotate(img_crop, 180);img_crop=fliplr(img_crop);
            end
        case 4
            if abs(theta_deg - 0) > 1 || abs(theta_deg - 180) > 1
                img_crop=fliplr(img_crop);
            end
    end
    % 构建3D贴图点
    %x1 = xt1(1); y1 = yt1(1);
    %x2 = xt1(end); y2 = yt1(end);
    x1 = p2(1); y1 = p2(2);  % 底边左端点
    x2 = p1(1); y2 = p1(2);  % 底边右端点

    v = [x2 - x1, y2 - y1]; v = v / norm(v);
    P0 = [x1, y1, 0]; P1 = [x2, y2, 0];
    P2 = P1 + [0, 0, H_real]; P3 = P0 + [0, 0, H_real];

    % 曲面贴图
    X = [P0(1), P1(1); P3(1), P2(1)];
    Y = [P0(2), P1(2); P3(2), P2(2)];
    Z = [P0(3), P1(3); P3(3), P2(3)];

    % 将 surf 添加到统一坐标轴 ax 中
surf(ax, X, Y, Z, flipud(img_crop), ...
    'FaceColor', 'texturemap', ...
    'EdgeColor', 'none');
end
