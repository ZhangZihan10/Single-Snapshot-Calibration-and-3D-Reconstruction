%% 机器人系统避障（纯色面板）   只进行3D重建（真实环境）加入相机旋转(成功）
%% 2. 遍历所有图像并进行透视变换和3D建模
clear

load('fisheyeCameraParamsReal2old.mat');
load('Omni_Calib_Results_Real2old2.mat');
ocam_model = calib_data.ocam_model;
intrinsics = cameraParams.Intrinsics;

% 激光参数
camX = 0.312; camY = 3.378; camZ = 0.193;
camX2 = -2.312; camY2 = 1.378; camZ2 = 0.193;
%lasX = -1.53; lasY = -2.407;
lasX = 0; lasY = 0;
CVsyst_rot = 0;
las_dist = 320.99;
%CVsyst_x = 1800;
%CVsyst_y = 3400;
CVsyst_x = 0;
CVsyst_y = 0;

Cube_l = 450;  % 底边实际长度（单位 mm）
fov = 159;%160
f_max = 611;  % 等效焦距610

% 获取图像列表
outputFolder = 'RobotBoard';
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

folderPath = outputFolder;
imageFiles = dir(fullfile(folderPath, '*.jpg'));

outputFolder2 = 'RobotBoardL';%激光识别文件
folderPath2 = outputFolder2;
imageFiles2 = dir(fullfile(folderPath2, '*.jpg'));

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
    %testImage = imresize(testImage, [1920, 1920]);
    filename2 = fullfile(folderPath2, imageFiles(i).name);
    testImage2 = imread(filename2);
    % 鱼眼矫正
    %correctedImg = undistortFisheyeImage(testImage, intrinsics, 'OutputView', 'same', 'ScaleFactor', 0.4);
    % figure;imshow(correctedImg);
    % 激光识别 + 坐标映射
    img = LaserFind(testImage2);
    %figure;imshow(img);
    %img = las_segm(testImage);%传统激光识别方法
    [x1, y1] = mappingr(img, camY, camX, camZ, lasY, lasX, las_dist, ocam_model);
    x1 = x1(2:end); y1 = y1(2:end);
    [xt1, yt1] = MappingRobotR([x1', y1'], Cube_l);
    % 对 xt1 和 yt1 进行一次线性拟合
    figure;scatter(xt1,yt1,5,'filled',"green");

  
    % 假设 xt1, yt1 是你的点列向量
    pts = [xt1(:), yt1(:)];  % N×2矩阵  
    %pts = [x1(:), y1(:)]; 
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
    points = [xt1(:), yt1(:)];  % 强制列向量确保维度一致
    %points = [x1(:), y1(:)];
    % --- 新增：清洗数据，剔除 Inf 和 NaN ---
    valid_idx = isfinite(points(:,1)) & isfinite(points(:,2));
    points = points(valid_idx, :);
    
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

    %第一次旋转
    yaw=camY2;pitch=camZ2;roll=camX2;
    %yaw=2.4;pitch=4.3;roll=-0.25;

    R = eul2rotm(deg2rad([yaw, pitch, roll]), 'ZYX');
    % 相机旋转
     % 根据图像序号设置姿态角
    switch i
        case 2
            roll = 0; pitch = 90; yaw = 90;
                if abs(theta_deg - 0) < 1 || abs(theta_deg - 180) < 1|| abs(theta_deg - 90) < 1
                    yaw2 = 0;
                elseif theta_deg>=90
                    yaw2 = 180-theta_deg;
                else
                    yaw2 = -(theta_deg);
                end
            R2 = eul2rotm(deg2rad([yaw, pitch,  -1]), 'ZYZ');
        case 1
            roll = 0; pitch = 0;
                if abs(theta_deg - 0) < 1 || abs(theta_deg - 180) < 1|| abs(theta_deg - 90) < 1
                    yaw = -90;
                elseif theta_deg>=90
                   yaw = 90-(theta_deg);
                else
                    yaw = -theta_deg;
                end
                
            R2 = eul2rotm(deg2rad([roll,pitch,yaw]), 'ZYX');
            %R2 = eul2rotm(deg2rad([roll,pitch,-90]), 'ZYX');
        case 4
            roll = 90; pitch = 0;
                if abs(theta_deg - 0) < 1 || abs(theta_deg - 180) < 1 || abs(theta_deg - 90) < 1
                    yaw = 0;
                elseif theta_deg<=90
                    yaw = -theta_deg;
                else
                    yaw = theta_deg-90;
                end
            R2 = eul2rotm(deg2rad([roll,pitch,yaw]), 'XYZ');
        case 3
            roll = 0; pitch = 90; yaw = -90;
                if abs(theta_deg - 0) < 1 || abs(theta_deg - 180) < 1|| abs(theta_deg - 90) < 1
                    yaw2 = -90;
                elseif theta_deg<=90
                    yaw2 = -theta_deg;
                else
                    yaw2 = -theta_deg+90;
                end
            R2 = eul2rotm(deg2rad([yaw2, pitch,  roll]), 'ZYX');
            %R2 = eul2rotm(deg2rad([-90, pitch,  roll]), 'ZYX');
        otherwise, roll = 0; pitch = -90; yaw = -90;
    end


    % ----------------- 5. 创建投影平面网格 -----------------
    Wout = 1920; Hout = 1080;
    aspect = Wout / Hout;
    
    halfFovY = deg2rad(fov)/2.2;        % 用标准半视场
    grid_scale_y = tan(halfFovY);
    grid_scale_x = aspect * grid_scale_y;
    
    [u, v] = meshgrid(linspace(-grid_scale_x, grid_scale_x, Wout), ...
                      linspace(-grid_scale_y, grid_scale_y, Hout));
    
    P = [u(:), ones(numel(u),1), v(:)]';

    P_rotated = R* R2 * P;
    theta = atan2(P_rotated(3,:), P_rotated(1,:));
    phi = atan2(vecnorm(P_rotated([1,3],:), 2, 1), P_rotated(2,:));

    theta_samples = linspace(0, pi/2, 1000);
    % 计算图像边缘最大半径
    %r_max = norm([imageSize(1)/2 - intrinsics.DistortionCenter(1), imageSize(2)/2 - intrinsics.DistortionCenter(2)]);
    
    % 定义鱼眼投影方程并求解θ_max
    %r_samples = polyval([intrinsics.MappingCoefficients(4),...
    %                    intrinsics.MappingCoefficients(3),...
    %                    intrinsics.MappingCoefficients(2),...
    %                    intrinsics.MappingCoefficients(1)], theta_samples);
    %theta_max = interp1(r_samples, theta_samples, r_max, 'pchip');
    
    % 计算等效焦距
    %f_max = r_max / theta_max;  % 替代原代码中的 intrinsics.FocalLength(1)
    %f_max = 619;    %611.6;
    maxFOV = 2 * f_max;         % 最大视场角（等效焦距法）

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
        perspectiveImg(:,:,c) = interp2(double(testImage(:,:,c)),...
            reshape(x,Hout, Wout), reshape(y,Hout, Wout), 'linear', 0);
    end

    % 提取非黑像素区域
    mask = ~(perspectiveImg(:,:,1)<45 & perspectiveImg(:,:,2)<45 & perspectiveImg(:,:,3)<45);
    %mask = any(perspectiveImg > 10, 3); % 阈值可根据图像调整
    % figure;imshow(perspectiveImg);

    [row_idx, col_idx] = find(mask);
    if isempty(row_idx)
        continue;
    end
    h_pixel = max(row_idx) - min(row_idx);
    w_pixel = max(col_idx) - min(col_idx);
    H_real = L * (h_pixel / w_pixel)+1.4;

    fprintf('高度: %.2f\n', H_real);

   % 裁剪贴图
    row_min = min(row_idx); row_max = max(row_idx);
    col_min = min(col_idx); col_max = max(col_idx);
    
    img_crop = perspectiveImg(row_min:row_max, col_min:col_max, :);
    
    % --- 新增：截取对应的透明度掩膜 (Alpha Mask) ---
    % 将 logical 转换为 double，1代表不透明，0代表透明
    alpha_crop = double(mask(row_min:row_max, col_min:col_max)); 
    
    figure; imshow(img_crop)
    
    switch i
        case 3
            if abs(theta_deg - 0) > 1 || abs(theta_deg - 180) > 1
                img_crop = imrotate(img_crop, 180); img_crop = fliplr(img_crop);
                % --- 同步翻转/旋转透明度掩膜 ---
                alpha_crop = imrotate(alpha_crop, 180); alpha_crop = fliplr(alpha_crop);
            end
        case 4
            if abs(theta_deg - 0) > 1 || abs(theta_deg - 180) > 1 
                img_crop = fliplr(img_crop);
                % --- 同步翻转透明度掩膜 ---
                alpha_crop = fliplr(alpha_crop);
            end
    end
    
    % 构建3D贴图点
    x1 = p2(1); y1 = p2(2);  % 底边左端点
    x2 = p1(1); y2 = p1(2);  % 底边右端点
    v = [x2 - x1, y2 - y1]; v = v / norm(v);
    P0 = [x1, y1, -4]; P1 = [x2, y2, -4];
    P2 = P1 + [0, 0, H_real]; P3 = P0 + [0, 0, H_real];
    
    % 曲面贴图坐标
    X = [P0(1), P1(1); P3(1), P2(1)];
    Y = [P0(2), P1(2); P3(2), P2(2)];
    Z = [P0(3), P1(3); P3(3), P2(3)];
    
    % --- 修改：将 surf 添加透明度属性 ---
    surf(ax, X, Y, Z, flipud(img_crop), ...
        'FaceColor', 'texturemap', ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 'texturemap', ...                % 开启面片透明度贴图
        'AlphaData', flipud(alpha_crop), ...          % 传入我们做好的透明度掩膜 (注意也要flipud)
        'AlphaDataMapping', 'none');                  % 确保 0 就是完全透明，1 就是完全不透明
end
