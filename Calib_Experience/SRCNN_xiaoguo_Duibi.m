%%  双三次插值+SRCNN 处理  不同方法角点检测效果对比


clc; clear; close all;


%% 原图 Matlab自带角点检测
%load('fisheyeCameraParams.mat');   % 含 cameraParams (fisheyeParameters)
%I = imread('test08.jpg');
I = imread('v49.jpg');

% 统一分辨率（应与标定时一致或相近）
%targetSize = [1920, 1920];

targetSize = [1080, 1920];
I = imresize(I, targetSize);

%intrinsicsIn = cameraParams.Intrinsics;

% ---------------- 2. 棋盘角点检测 ----------------
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

hold off;


%% 原图 IVAN 方法
%load('Omni_Calib_Results_Sim.mat'); % Calib parameters
%load('Omni_Calib_Results_Real.mat');
load('Omni_Calib_Results_Real2old2.mat');
ocam_model = calib_data.ocam_model; % Calib parameters
i = calib_data.n_ima;
calib_data.L(i+1)={'v49.jpg'};
%calib_data.L{i+1} = enhanced_name;  I = imread('c_2.jpg');
kk=i+1;

use_corner_find=1;
[cb, Xp, Yp] = get_checkerboard_cornersUrban(kk, use_corner_find, calib_data);

% 显示原图
figure; imshow(I); hold on;

% 绘制角点位置
plot(Yp, Xp, 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);

% 标注角点序号（可选）
for i = 1:size(Yp)
    text(Yp(i)+5, Xp(i), num2str(i), ...
         'Color','y','FontSize',8,'FontWeight','bold');
end


hold off;


%% 图像增强处理后 Matlab自带
%图像增强
%img=imread('e0.jpg'); up_scale=3; im_h = SRCNN_single_image(img, up_scale);enhanced_name = 'e_0.jpg';imwrite(im_h, enhanced_name);

%load('fisheyeCameraParams.mat');   % 含 cameraParams (fisheyeParameters)
%I = imread('test08.jpg'); I2 = imread('testr3_h.jpg');figure; imshow(I2);
I2 = imread('v_49.jpg');

% 统一分辨率（应与标定时一致或相近）
%targetSize = [1920, 1920];

targetSize = [1080, 1920];
I2 = imresize(I2, targetSize);

%intrinsicsIn = cameraParams.Intrinsics;

% ---------------- 2. 棋盘角点检测 ----------------
[imagePoints, boardSize] = detectCheckerboardPoints(I2);
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


hold off;

%% 增强  IVAN 方法
%load('Omni_Calib_Results_Sim.mat'); % Calib parameters
%load('Omni_Calib_Results_Real.mat');
load('Omni_Calib_Results_Real2old2.mat');
ocam_model = calib_data.ocam_model; % Calib parameters
i = calib_data.n_ima;
calib_data.L(i+1)={'v_49.jpg'};
%calib_data.L{i+1} = enhanced_name;  I = imread('c_2.jpg');
kk=i+1;

use_corner_find=1;
[cb, Xp, Yp] = get_checkerboard_cornersUrban(kk, use_corner_find, calib_data);

% 显示原图
figure; imshow(I); hold on;

% 绘制角点位置
plot(Yp, Xp, 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);

% 标注角点序号（可选）
for i = 1:size(Yp)
    text(Yp(i)+5, Xp(i), num2str(i), ...
         'Color','y','FontSize',8,'FontWeight','bold');
end


hold off;

%%   Ours
%load('Omni_Calib_Results_Sim.mat'); % Calib parameters
%load('Omni_Calib_Results_Real.mat');
load('Omni_Calib_Results_Real2old2.mat');
ocam_model = calib_data.ocam_model; % Calib parameters
i = calib_data.n_ima;
calib_data.L(i+1)={'v_49.jpg'};
%calib_data.L{i+1} = enhanced_name;  I = imread('c_2.jpg');
kk=i+1;

use_corner_find=1;
[cb, Xp, Yp] = get_checkerboard_cornersUrban10_1(kk, use_corner_find, calib_data);

% 显示原图
figure; imshow(I); hold on;

% 绘制角点位置
plot(Yp, Xp, 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);

% 标注角点序号（可选）
for i = 1:size(Yp)
    text(Yp(i)+5, Xp(i), num2str(i), ...
         'Color','y','FontSize',8,'FontWeight','bold');
end


hold off;
