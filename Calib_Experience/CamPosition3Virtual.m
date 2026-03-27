
%%对比实验 虚拟 单张校准相机，激光面，定位    先转换为透视相机再计算姿态
%% =======================================================
%  路线 B：鱼眼像素去畸变 → 针孔像素 → PnP 姿态估计
%  输出相机欧拉角 (roll, pitch, yaw) 与世界坐标位置
% =======================================================

clc; clear; close all;

%% ---------------- 1. 加载标定参数与图像 ----------------
load('fisheyeCameraParams.mat');   % 含 cameraParams (fisheyeParameters)
%I = imread('test08.jpg');
I = imread('test08E.jpg');

% 统一分辨率（应与标定时一致或相近）
targetSize = [1920, 1920];
I = imresize(I, targetSize);

intrinsicsIn = cameraParams.Intrinsics;

%% ---------------- 2. 棋盘角点检测 ----------------
[imagePoints, boardSize] = detectCheckerboardPoints(I);
if isempty(imagePoints)
    error('未检测到棋盘角点，请检查输入图像或棋盘尺寸。');
end

% 显示原图
figure; imshow(I); hold on;

% 绘制角点位置
plot(imagePoints(:,1), imagePoints(:,2), 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);

% 标注角点序号（可选）
for i = 1:size(imagePoints,1)
    text(imagePoints(i,1)+5, imagePoints(i,2), num2str(i), ...
         'Color','y','FontSize',8,'FontWeight','bold');
end

% ================== 绘制棋盘局部坐标轴 ==================

hold off;

squareSize = 26; % 棋盘格边长 (mm)
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

fprintf('✅ 检测到 %d 个棋盘角点。\n', size(imagePoints,1));

%% ---------------- 3. 鱼眼去畸变 → 针孔像素 ----------------

OUTPUT_VIEW = 'same';       % 'same' 或 'full'           % 0~1，数字小=更广角（通常 1 就好）
scaleFactor = 1;                      % 或者 [0.9 0.9] 之类%scaleFactor = double(1.0);   % ✅ 显式定义为 double
undistortedPts = undistortFisheyePoints(imagePoints, intrinsicsIn);

% 可视化去畸变图像
[J, intrinsicsOut] = undistortFisheyeImage(I, intrinsicsIn, ...
    'OutputView', 'same', 'ScaleFactor', 1);
figure(1);
subplot(1,2,1); imshow(I); title('原始鱼眼图');
subplot(1,2,2); imshow(J); title('去畸变(针孔)图');

%% ---------------- 4. 使用真实K执行PnP ----------------
% 角点像素（已去畸变）
undistortedPts = double(undistortedPts);      % N×2

% 棋盘世界坐标：N×2 -> N×3（Z=0 平面）
worldPoints2d = generateCheckerboardPoints(boardSize, squareSize);  % N×2, 单位mm
worldPoints3d = [worldPoints2d, zeros(size(worldPoints2d,1), 1)];   % N×3

% PnP
[rotMat, transVec, inlierIdx] = estimateWorldCameraPose( ...
    undistortedPts, worldPoints3d, intrinsicsOut, ...
    'MaxReprojectionError', 2.0, 'Confidence', 99.99, 'MaxNumTrials', 3000);

fprintf('\n=== 相机姿态估计完成 ===\n');
disp('旋转矩阵 R (world→camera):');
disp(rotMat);
disp('平移向量 T (mm):');
disp(transVec);

%% ---------------- 5. 计算欧拉角与相机位置 ----------------
% MATLAB定义: R = Rz(yaw) * Ry(pitch) * Rx(roll)
% 对应旋转顺序为 ZYX
eulZYX = rotm2eul(rotMat);        % [yaw pitch roll] (radians)
yaw_deg   = rad2deg(eulZYX(1));
pitch_deg = rad2deg(eulZYX(2));
roll_deg  = rad2deg(eulZYX(3));

% 将相机位置换算到世界坐标（世界->相机的逆）
R_WC = rotMat';                   % camera->world
C_W  = -R_WC * transVec';         % 相机在世界坐标下的位置 (mm)

fprintf('\n=== 相机外参结果 (世界坐标系) ===\n');
fprintf('相机位置 C_W = [%.2f, %.2f, %.2f] mm\n', C_W(1), C_W(2), C_W(3));
fprintf('相机欧拉角 (deg): roll = %.2f°, pitch = %.2f°, yaw = %.2f°\n', ...
        roll_deg, pitch_deg, yaw_deg);

%% ---------------- 6. 可视化结果 ----------------
% 将 world->camera 的 R,T 转为 camera->world 的姿态与位置
R_WC = rotMat.';                 % camera -> world
C_W  = -R_WC * transVec.';       % 相机中心在世界系 (列向量)

figure; hold on; grid on; axis equal; view(35,20);
xlabel X; ylabel Y; zlabel Z;

% 画相机
pcshow(worldPoints3d,'VerticalAxis','Y','VerticalAxisDir','down', ...
     'MarkerSize',30);
plotCamera('Size',30, 'Orientation', R_WC, ...
           'Location', transVec,'Color', 'r', 'Opacity', 0.1);

title('Camera pose (plotCamera)');


% 重投影可视化
reprojPts = worldToImage(intrinsicsOut, rotMat, transVec, worldPoints3d);
figure(3);
imshow(J, []); hold on;
plot(undistortedPts(:,1), undistortedPts(:,2), 'go', 'MarkerSize',5, 'LineWidth',1.2);
plot(reprojPts(:,1), reprojPts(:,2), 'r+', 'MarkerSize',5, 'LineWidth',1.2);
legend({'去畸变角点','重投影点'});
title('PnP重投影误差对比');
