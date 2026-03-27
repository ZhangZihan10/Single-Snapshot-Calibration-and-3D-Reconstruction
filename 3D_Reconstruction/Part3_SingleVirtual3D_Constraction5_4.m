%% （纯色盒子）进行3D重建（真实环境）加入相机旋转(成功）高度矫正
%% 正方体盒子3D重建（单文件处理：正面与顶面拼接）
clear
load('fisheyeCameraParamsReal2old.mat');
load('Omni_Calib_Results_Real2old1.mat');
ocam_model = calib_data.ocam_model;
intrinsics = cameraParams.Intrinsics;


tic;
% 激光与相机参数
camX = 0.312; camY = 3.378; camZ = 4.193;
lasX = -1.53; lasY = -2.407;
camX2 = -0.312; camY2 = 0.378; camZ2 = 1.193;
CVsyst_rot = 0;
las_dist = 320.99;
CVsyst_x = 0;
CVsyst_y = 0;
Cube_l = 450;  % 底边实际长度（单位 mm）
fov = 159;     % 160
f_max = 611;   % 等效焦距610

% ================= 1. 指定文件路径 =================
% 这里将循环去掉，直接指定正面、顶面和激光源文件
filename_laser = 'WBoxL/ObstacleBoardL_1.jpg'; % 用于提取基准底边
filename_front = 'WBoxR/ObstacleBoard_1.jpg';       % 正面抠图文件（请修改为你的实际路径）
filename_top   = 'WBoxR/ObstacleBoard_2.jpg';         % 顶面抠图文件（请修改为你的实际路径）

% =================================================

% 初始化绘图
figure(99); clf;  
ax = axes; hold(ax, 'on');   
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
axis(ax, 'equal'); axis(ax, [0 25 0 25 0 25]);
grid(ax, 'on'); view(ax, 3);
title(ax, '正方体正面与顶面 3D 拼接结果');

% ================= 2. 激光识别与基准计算 =================
testImageLaser = imread(filename_laser);
img_las = LaserFind(testImageLaser);
[x1_las, y1_las] = mappingr(img_las, camY, camX, camZ, lasY, lasX, las_dist, ocam_model);
x1_las = x1_las(2:end); y1_las = y1_las(2:end);

pts = [x1_las(:), y1_las(:)]; 
valid_idx = isfinite(pts(:,1)) & isfinite(pts(:,2));
pts = pts(valid_idx, :);

meanPt = mean(pts, 1);    
pts_centered = pts - meanPt;
[~, ~, V] = svd(pts_centered, 0);   
dir_vector = V(:,1);  

v_y = [0, 1];  
dir = dir_vector(:)' / norm(dir_vector);
cos_theta = max(min(dot(v_y, dir), 1), -1); 
theta_deg = rad2deg(acos(cos_theta));
if dir(1) < 0
    theta_deg = 180 - theta_deg;
end
fprintf('线段与 Y 轴的夹角：%.2f°\n', theta_deg);

L = 0; p1 = [0, 0]; p2 = [0, 0];
N = size(pts, 1);
for i1 = 1:N
    for j = i1+1:N
        d = norm(pts(i1,:) - pts(j,:));  
        if d > L
            L = d; p1 = pts(i1,:); p2 = pts(j,:);
        end
    end
end
fprintf('底边(激光)线段长度: %.2f\n', L);

% ================= 3. 姿态角计算 (Case 2 逻辑) =================
yaw1 = camY2; pitch1 = camZ2; roll1 = camX2;
R = eul2rotm(deg2rad([yaw1, pitch1, roll1]), 'ZYX');

roll2 = 0; pitch2 = 0;
if abs(theta_deg - 0) < 1 || abs(theta_deg - 180) < 1 || abs(theta_deg - 90) < 1
    yaw2 = -90;
elseif theta_deg >= 90
    yaw2 = 90 - theta_deg;
else
    yaw2 = -theta_deg;
end
R2 = eul2rotm(deg2rad([roll2, pitch2, yaw2]), 'ZYX');

% ================= 4. 构建透视投影网格 =================
Wout = 1920; Hout = 1080;
aspect = Wout / Hout;
halfFovY = deg2rad(fov)/2.9;        
grid_scale_y = tan(halfFovY);
grid_scale_x = aspect * grid_scale_y;

[u, v] = meshgrid(linspace(-grid_scale_x, grid_scale_x, Wout), ...
                  linspace(-grid_scale_y, grid_scale_y, Hout));
P = [u(:), ones(numel(u),1), v(:)]';
P_rotated = R * R2 * P;
P_rotated2 = R * P;

theta = atan2(P_rotated(3,:), P_rotated(1,:));
phi = atan2(vecnorm(P_rotated([1,3],:), 2, 1), P_rotated(2,:));

I = (2 * phi .* cos(theta)) / deg2rad(fov);
J = (2 * phi .* sin(theta)) / deg2rad(fov);
S_inv = inv(intrinsics.StretchMatrix);
I_corrected = S_inv(1,1)*I + S_inv(1,2)*J;
J_corrected = S_inv(2,1)*I + S_inv(2,2)*J;
x_proj = I_corrected * f_max + intrinsics.DistortionCenter(1);
y_proj = J_corrected * f_max + intrinsics.DistortionCenter(2);

% ================= 5. 处理正面抠图 =================
testImageFront = imread(filename_front);
perspImg_front = zeros(size(testImageFront), 'like', testImageFront);
for c = 1:3
    perspImg_front(:,:,c) = interp2(double(testImageFront(:,:,c)),...
        reshape(x_proj,Hout, Wout), reshape(y_proj,Hout, Wout), 'linear', 0);
end
mask_f = ~(perspImg_front(:,:,1)<25 & perspImg_front(:,:,2)<25 & perspImg_front(:,:,3)<25);
[row_f, col_f] = find(mask_f);

h_pixel_f = max(row_f) - min(row_f);
w_pixel_f = max(col_f) - min(col_f);
H_front = L * (h_pixel_f / w_pixel_f)+20;  % Case 2 的高度补偿

img_crop_f = perspImg_front(min(row_f):max(row_f), min(col_f):max(col_f), :);
alpha_crop_f = double(mask_f(min(row_f):max(row_f), min(col_f):max(col_f)));
figure;imshow(img_crop_f);
% ================= 6. 处理顶面抠图 =================
theta = atan2(P_rotated2(3,:), P_rotated2(1,:));
phi = atan2(vecnorm(P_rotated2([1,3],:), 2, 1), P_rotated2(2,:));

I = (2 * phi .* cos(theta)) / deg2rad(fov);
J = (2 * phi .* sin(theta)) / deg2rad(fov);
S_inv = inv(intrinsics.StretchMatrix);
I_corrected = S_inv(1,1)*I + S_inv(1,2)*J;
J_corrected = S_inv(2,1)*I + S_inv(2,2)*J;
x_proj = I_corrected * f_max + intrinsics.DistortionCenter(1);
y_proj = J_corrected * f_max + intrinsics.DistortionCenter(2);

testImageTop = imread(filename_top);
perspImg_top = zeros(size(testImageTop), 'like', testImageTop);
for c = 1:3
    perspImg_top(:,:,c) = interp2(double(testImageTop(:,:,c)),...
        reshape(x_proj,Hout, Wout), reshape(y_proj,Hout, Wout), 'linear', 0);
end
mask_t = ~(perspImg_top(:,:,1)<25 & perspImg_top(:,:,2)<25 & perspImg_top(:,:,3)<25);
[row_t, col_t] = find(mask_t);

h_pixel_t = max(row_t) - min(row_t);
w_pixel_t = max(col_t) - min(col_t);
W_top = L * (h_pixel_t / w_pixel_t);         % 顶面纵深

img_crop_t = perspImg_top(min(row_t):max(row_t), min(col_t):max(col_t), :);
alpha_crop_t = double(mask_t(min(row_t):max(row_t), min(col_t):max(col_t)));
figure;imshow(img_crop_t);
% ================= 7. 3D 坐标拼接与贴图 =================
% (1) 准备基准点
fx1 = p2(1); fy1 = p2(2); 
fx2 = p1(1); fy2 = p1(2);

% (2) 绘制正面 (立面)
X_f = [fx1, fx2; fx1, fx2];
Y_f = [fy1, fy2; fy1, fy2];
Z_f = [0, 0; H_front, H_front];

surf(ax, X_f, Y_f, Z_f, flipud(img_crop_f), ...
    'FaceColor', 'texturemap', 'EdgeColor', 'none', ...
    'FaceAlpha', 'texturemap', 'AlphaData', flipud(alpha_crop_f), ...
    'AlphaDataMapping', 'none');

% (3) 绘制顶面 (平面向后延伸)
v_edge = [fx2 - fx1, fy2 - fy1];
v_edge = v_edge / norm(v_edge);
v_normal = [-v_edge(2), v_edge(1)]; % 法向量，确保向后延伸

tx3 = fx1 + v_normal(1) * W_top; 
ty3 = fy1 + v_normal(2) * W_top;
tx4 = fx2 + v_normal(1) * W_top; 
ty4 = fy2 + v_normal(2) * W_top;

X_t = [fx1, fx2; tx3, tx4];
Y_t = [fy1-1, fy2-1; ty3-1, ty4-1];
%H_front=H_front-5;
Z_t = [H_front, H_front; H_front, H_front];

surf(ax, X_t, Y_t, Z_t, flipud(img_crop_t), ...
    'FaceColor', 'texturemap', 'EdgeColor', 'none', ...
    'FaceAlpha', 'texturemap', 'AlphaData', flipud(alpha_crop_t), ...
    'AlphaDataMapping', 'none');

fprintf('3D拼接完成！正面高度: %.2f, 顶面纵深: %.2f\n', H_front, W_top);

elapsedTime = toc; % 将时间存入 elapsedTime 变量

fprintf('核心代码段共耗时: %.4f 秒\n', elapsedTime);