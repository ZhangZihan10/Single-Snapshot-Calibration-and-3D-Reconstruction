
%% 鱼眼矫正+旋转+去透视变换完整代码（成功）
%% 主要用于鱼眼图像还原
clc; clear; close all;

% ----------------- 1. 加载标定参数与图像 -----------------
load('fisheyeCameraParams.mat'); % 包含intrinsics和distortionCoeffs
testImage = imread('a7_1.jpg'); 
testImage = imresize(testImage, [1920, 1920]); % 统一分辨率
imageSize = [1920, 1920];


%load('fisheyeCameraParams.mat', 'fisheyeParams'); 
intrinsics = cameraParams.Intrinsics;  % 访问Intrinsics子对象
% ----------------- 2. 鱼眼畸变校正 -----------------
correctedImg = undistortFisheyeImage(testImage, intrinsics,...
    'OutputView', 'same', 'ScaleFactor', 0.5); % 扩展视野
%correctedImg = undistortFisheyeImage(testImage, intrinsics, 'OutputView', 'same', 'ScaleFactor', 1);


%% 激光识别
img=LaserFind(testImage);%新激光识别方法
%img = las_segm(image);%传统激光识别方法
% Mapping
load('Omni_Calib_Results_Unity.mat'); % Calib parameters
ocam_model = calib_data.ocam_model; % Calib parameters
camX =0;%-2.5; % Camera parameters
camY =0;%6; % Camera parameters
camZ =0;% 3; % Camera parameters
lasX = 0;%1.5; % Laser Plane parameters
lasY = 0;%-2.5; % Laser Plane parameters
CVsyst_rot = 0; % CV System initial rotation
las_dist = 4950; % Laser Plane parameters
CVsyst_x = 1800; % CV System initial position 在unity中为CVSystemOrigin的位置参数z*1000
CVsyst_y = 3400; % CV System initial position 在unity中为CVSystemOrigin的位置参数x*-1000
[x1,y1] = mapping(img,CVsyst_rot,CVsyst_x,CVsyst_y,camX,camY,camZ,lasX,lasY,...
    las_dist,ocam_model); % mapping function
% Finally figure:
figure;
scatter(x1,y1,5,'filled'); % Laser intersections
hold on;
plot(CVsyst_x,CVsyst_y,'r*'); % CV System location
grid on;
%机器人坐标系
Cube_l=450;
C_Up=[x1',y1'];
[xt1,yt1]=MappingRobot(C_Up,Cube_l);




%% 图像提取
% ----------------- 3. 定义虚拟相机旋转参数 -----------------
yaw = 0;   % Z轴旋转角度（论文Table5-4左视图）90 y
pitch = 60; % 根据实际安装角度调整z，当物体与相机不水平时先旋转z，再旋转x
roll= 90;   %90;x 前后视图
fov = 115;   % 视场角（立方体贴图标准）

% ----------------- 4. 构建旋转矩阵 -----------------
%R = eul2rotm(deg2rad([yaw, 0, 0]), 'ZYX'); % 绕Z轴优先旋转

% ----------------- 5. 创建投影平面网格 -----------------
theta_max1 = deg2rad(fov) / 2; % 转换为弧度并取半角（59°）
% 根据theta_max计算网格范围（覆盖完整视场）
grid_scale = tan(theta_max1) * 2; % 增加10%余量防止黑边

[u, v] = meshgrid(linspace(-grid_scale, grid_scale, 1920),...
                  linspace(-grid_scale, grid_scale, 1920));
%[u, v] = meshgrid(linspace(-2, 3, 1920), linspace(-2, 3, 1920));
P = [u(:), ones(numel(u),1), v(:)]'; % 对应论文公式(5.8)

% ----------------- 6. 应用旋转变换 -----------------

R = eul2rotm(deg2rad([yaw, pitch, roll]), 'ZYX');
P_rotated = R * P; % 对应论文公式(5.9)向量旋转

% ----------------- 7. 计算鱼眼坐标映射 -----------------
theta = atan2(P_rotated(3,:), P_rotated(1,:)); % 方位角
phi = atan2(vecnorm(P_rotated([1,3],:), 2, 1), P_rotated(2,:)); % 俯仰角

theta_samples = linspace(0, pi/2, 1000);
% 计算图像边缘最大半径
r_max = norm([imageSize(1)/2 - intrinsics.DistortionCenter(1), imageSize(2)/2 - intrinsics.DistortionCenter(2)]);

% 定义鱼眼投影方程并求解θ_max
r_samples = polyval([intrinsics.MappingCoefficients(4),...
                    intrinsics.MappingCoefficients(3),...
                    intrinsics.MappingCoefficients(2),...
                    intrinsics.MappingCoefficients(1)], theta_samples);
theta_max = interp1(r_samples, theta_samples, r_max, 'pchip');

% 计算等效焦距
%f_max = r_max / theta_max;  % 替代原代码中的 intrinsics.FocalLength(1)
f_max = 611.6;    %611.6;
maxFOV = 2 * f_max;         % 最大视场角（等效焦距法）

% 转换为鱼眼归一化坐标
%maxFOV = 2*intrinsics.FocalLength(1); % 等效f_max
I = (2*phi.*cos(theta))/deg2rad(fov); 
J = (2*phi.*sin(theta))/deg2rad(fov);
%I = (2 * phi .* cos(theta)) / deg2rad(f_max); 
%J = (2 * phi .* sin(theta)) / deg2rad(f_max); 

% ----------------- 8. 坐标反投影到原图 -----------------
%x = (I*intrinsics.FocalLength(1)) + intrinsics.PrincipalPoint(1);
%y = (J*intrinsics.FocalLength(2)) + intrinsics.PrincipalPoint(2);
% 修正代码（严格匹配表格参数）
S_inv = inv(intrinsics.StretchMatrix);
I_corrected = S_inv(1,1)*I + S_inv(1,2)*J;
J_corrected = S_inv(2,1)*I + S_inv(2,2)*J;
x = I_corrected * f_max + intrinsics.DistortionCenter(1);
y = J_corrected * f_max + intrinsics.DistortionCenter(2);

%x = I_corrected * f_max+1200 ;
%y = J_corrected * f_max+1200 ;
% ----------------- 9. 双线性插值重采样 -----------------
perspectiveImg = zeros(size(testImage), 'like', testImage);
for c = 1:3
    perspectiveImg(:,:,c) = interp2(double(testImage(:,:,c)),...
        reshape(x,1920,1920), reshape(y,1920,1920), 'linear', 0);
end


% 生成二值掩膜（过滤纯黑背景）
mask = any(perspectiveImg > 10, 3); % 阈值可根据图像调整
% 替代方案：
stats = regionprops(mask, 'BoundingBox');
bbox = stats.BoundingBox; % 格式：[x_min, y_min, width, height]

% 转换为行列索引
x_min = floor(bbox(1)) + 1;
y_min = floor(bbox(2)) + 1;
x_max = x_min + floor(bbox(3)) - 1;
y_max = y_min + floor(bbox(4)) - 1;

% 裁剪图像
perspectiveImg = perspectiveImg(y_min:y_max, x_min:x_max, :);

% ----------------- 10. 可视化对比 -----------------
figure;
subplot(1,3,1); imshow(testImage); title('原始鱼眼图像');
subplot(1,3,2); imshow(correctedImg); title('鱼眼校正图像');
subplot(1,3,3); imshow(perspectiveImg); title('去透视');
imwrite(perspectiveImg, 'wall_final.jpg');