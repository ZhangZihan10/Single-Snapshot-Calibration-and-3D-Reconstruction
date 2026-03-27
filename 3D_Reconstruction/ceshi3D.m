%% 加神经网络语义分割

clear
load 3DtrainedNetresnet50-10.mat;%trainedNet1为resnet18模型，trainedNet3为resnet50模型
%[file,path]=uigetfile('D:\桌面文件夹\robot course\arduino\视觉识别\语义分割虚拟\测试图片\');
%filepath=fullfile(path,file);
I=imread('a5.jpg');
%cam=webcam(2);
%preview(cam);
%cam.Resolution='1920x1080';
%cam.Brightness=-10;%调整相机亮度
%I =snapshot(cam);

figure(1);
imshow(I);

I=imresize(I,[1920, 1920]);%imresize(I,[1080, 1080]);

C=semanticseg(I,net,'MiniBatchSize', 32);
%pxds =pixelLabelDatastore(I,classes,labelIDs);
%classes=["green","red", "blue","background"];%["red", "blue","green","background"];
classes=["platform","obstacleboard","cube"];%["red", "blue","green","background"];
%classes=["Bei","Red", "Green","Black","Grey"];
cmap=camvidColorMap;%需要更改内参数
B=labeloverlay(I,C,'ColorMap',cmap,'Transparency',0.4);
figure(2);
imshow(B),title("Semantic segmentation Result");
pixelLabelColorbar(cmap,classes);

% 创建 mask：只保留 cube 区域
cubeMask = C == 'obstacleboard';

% 将 mask 扩展为 3 通道（与 RGB 对应）
cubeMask3Channel = repmat(cubeMask, [1, 1, 3]);

% 将原始图像中非 cube 区域置为黑色
cubeOnlyImage = I; % 复制原图
cubeOnlyImage(~cubeMask3Channel) = 0; % 非 cube 区域设为 0

% 显示结果
figure;
imshow(cubeOnlyImage);
title('Only Cube Region');

%% 进行独立提取
% 转换为灰度图
gray = rgb2gray(cubeOnlyImage);

% 创建二值掩码（障碍板区域为白）
bw = imbinarize(gray);

% 去除小区域
cleanMask = bwareaopen(bw, 500);

% 连通区域分析
cc = bwconncomp(cleanMask);
labeled = labelmatrix(cc);

% 创建保存文件夹
outputFolder = 'obstacle_boardsA';
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

% 遍历每个障碍板区域并保存
for i = 1:cc.NumObjects
    % 提取当前障碍板掩码
    singleMask = ismember(labeled, i);
    singleMask3 = repmat(singleMask, [1, 1, 3]);

    % 创建只包含当前障碍板的图像
    singleBoardImage = zeros(size(cubeOnlyImage), 'uint8');
    singleBoardImage(singleMask3) = cubeOnlyImage(singleMask3);

    % 保存图像
    filename = fullfile(outputFolder, ['ObstacleBoard_' num2str(i) '.jpg']);
    imwrite(singleBoardImage, filename);
end

disp(['Saved ', num2str(cc.NumObjects), ' obstacle board images to folder: ', outputFolder]);



%% 鱼眼矫正+旋转+去透视变换完整代码（成功）

% ----------------- 1. 加载标定参数与图像 -----------------
load('fisheyeCameraParams.mat'); % 包含intrinsics和distortionCoeffs
% 图像所在的文件夹
folderPath = 'C:\arduino\MatlabProjectCV_Virtual_singleCamera\obstacle_boardsA';
% 获取该文件夹下所有 PNG 图像文件
imageFiles = dir(fullfile(folderPath, '*.jpg'));
% 遍历所有图像
%for i = 1:length(imageFiles)
    % 构建完整路径


    filename = fullfile(folderPath, imageFiles(1).name);
    
    % 读取图像
    %img = imread(filename);
    
    % 显示或处理图像
    %figure;
    %imshow(img);
    %title(['Image ', num2str(i), ': ', imageFiles(i).name]);
    
    % 你可以在这里添加你的处理代码...
%end
testImage =imread(filename);% cubeOnlyImage; %testImage = imread('a1.jpg');
testImage = imresize(testImage, [1920, 1920]); % 统一分辨率
imageSize = [1920, 1920];


%load('fisheyeCameraParams.mat', 'fisheyeParams'); 
intrinsics = cameraParams.Intrinsics;  % 访问Intrinsics子对象
% ----------------- 2. 鱼眼畸变校正 -----------------
correctedImg = undistortFisheyeImage(testImage, intrinsics,...
    'OutputView', 'same', 'ScaleFactor', 0.4); % 扩展视野
%correctedImg = undistortFisheyeImage(testImage, intrinsics, 'OutputView', 'same', 'ScaleFactor', 1);


%% 激光识别
img=LaserFind(testImage);%新激光识别方法
%img = las_segm(testImage);%传统激光识别方法
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
x1 = x1(2:end);
y1 = y1(2:end);

figure;
scatter(x1,y1,5,'filled'); % Laser intersections
hold on;
plot(CVsyst_x,CVsyst_y,'r*'); % CV System location
grid on;
%机器人坐标系
Cube_l=450;
C_Up=[x1',y1'];
[xt1,yt1]=MappingRobot(C_Up,Cube_l);

% 假设 xt1, yt1 是你的点列向量
pts = [xt1(:), yt1(:)];  % N×2矩阵
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
% 6. 绘图验证
figure;
scatter(xt1, yt1, 20, 'filled'); hold on;
plot(x_line, y_line, 'r-', 'LineWidth', 2);
xlabel('X'); ylabel('Y');
title('基于SVD的总最小二乘拟合直线');
grid on;
axis equal;
xlim([0 20]);
ylim([0 20]);
legend('原始点','拟合直线');
% 与 Y 轴的带方向夹角（正为逆时针，负为顺时针）
v_ref = [0, 1];
angle_signed_rad = atan2( ...
    det([v_ref; dir_vector']), ...
    dot(v_ref, dir_vector') ...
);
theta_deg_signed = rad2deg(angle_signed_rad);
fprintf('拟合直线与Y轴夹角（带符号）：%.2f 度\n', theta_deg_signed);


% 计算线段的长度
% 初始化最大距离
L = 0;
p1 = [0, 0];
p2 = [0, 0];

% 将点集合组合为二维坐标点矩阵
points = [xt1(:), yt1(:)];  % 强制列向量确保维度一致
N = size(points, 1);

% 双重循环查找最远两点
for i = 1:N
    for j = i+1:N
        d = norm(points(i,:) - points(j,:));  % 欧几里得距离
        if d > L
            L = d;
            p1 = points(i,:);
            p2 = points(j,:);
        end
    end
end
fprintf('线段长度: %.2f\n', L);


%% ----------------- 3. 定义虚拟相机旋转参数 -----------------
roll= 0;   %90;x 
pitch =90; % Y 60
yaw = 90;   % Z


yaw2=45;
fov = 115;   % 视场角（立方体贴图标准）

% ----------------- 4. 构建旋转矩阵 -----------------
%R = eul2rotm(deg2rad([yaw, 0, 0]), 'ZYX'); % 绕Z轴优先旋转

% ----------------- 5. 创建投影平面网格 -----------------
theta_max1 = deg2rad(fov) / 2; % 转换为弧度并取半角（59°）
% 根据theta_max计算网格范围（覆盖完整视场）
grid_scale = tan(theta_max1) * 4; % 增加10%余量防止黑边

[u, v] = meshgrid(linspace(-grid_scale, grid_scale, 1920),...
                  linspace(-grid_scale, grid_scale, 1920));
%[u, v] = meshgrid(linspace(-2, 3, 1920), linspace(-2, 3, 1920));
P = [u(:), ones(numel(u),1), v(:)]'; % 对应论文公式(5.8)

% ----------------- 6. 应用旋转变换 -----------------

%R = eul2rotm(deg2rad([yaw, pitch, roll]), 'ZYX');
                                   %R = eul2rotm(deg2rad([roll,pitch,yaw ]), 'XYZ');
R = eul2rotm(deg2rad([yaw, pitch, yaw2]), 'ZYZ');  % intrinsic ZYZ：先Z再Y再Z（每次都在新轴上）

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
mask = any(perspectiveImg > 0, 3); % 阈值可根据图像调整
% 替代方案：
stats = regionprops(mask, 'BoundingBox');
bbox = stats.BoundingBox; % 格式：[x_min, y_min, width, height]

% 转换为行列索引
x_min = floor(bbox(1)) + 1;
y_min = floor(bbox(2)) + 1;
x_max = x_min + floor(bbox(3)) - 1;
y_max = y_min + floor(bbox(4)) - 1;

% 裁剪图像
%perspectiveImg = perspectiveImg(y_min:y_max, x_min:x_max, :);

% ----------------- 10. 可视化对比 -----------------
figure;
subplot(1,3,1); imshow(testImage); title('原始鱼眼图图像');
subplot(1,3,2); imshow(correctedImg); title('鱼眼校正图像提取像');
subplot(1,3,3); imshow(perspectiveImg); title('去透视');
imwrite(perspectiveImg, 'wall_final.jpg');


%% 障碍板高度计算
% 1. 读取图像
img = perspectiveImg;
% 2. 构建非黑色掩码
non_black_mask = ~(img(:,:,1)<45 & img(:,:,2)<45 & img(:,:,3)<45);
%imshow(non_black_mask);
% 3. 提取非黑像素的坐标
[row_idx, col_idx] = find(non_black_mask);

% 4. 计算像素高度和底边宽度
h_pixel = max(row_idx) - min(row_idx);  % 像素高度
w_pixel = max(col_idx) - min(col_idx);  % 像素底边长度


% 6. 根据像素比例换算真实高度
H_real = L * (h_pixel / w_pixel);

% 7. 输出结果
fprintf('障碍板像素高度：%d px\n', h_pixel);
fprintf('障碍板像素底边宽度：%d px\n', w_pixel);
fprintf('障碍板真实高度：%.2f cm\n', H_real);


%% 3D重构

row_min = min(row_idx);
row_max = max(row_idx);
col_min = min(col_idx);
col_max = max(col_idx);

img_crop = img(row_min:row_max, col_min:col_max, :);

img_crop_rotated = imrotate(img_crop, 180);

%imshow(img_crop_rotated);
% 1. 实际障碍板尺寸
L_real = L;  % mm 底边长度

% 2. 构建底边方向向量
%x1 = xt1(1); y1 = yt1(1);  % 底边左端点
x1 = p2(1); y1 = p2(2);  % 底边左端点
x2 = p1(1); y2 = p1(2);  % 底边右端点

v = [x2 - x1, y2 - y1];  % 底边方向向量
v = v / norm(v);         % 单位化

% 3. 法向量方向（Z轴上升）
%n = [0, 0, 1];
% 4.四个角点：左下、右下、右上、左上（在 X-Z 平面内，Y 延伸）
% 底边两个点
P0 = [x1, y1, 0];  % 左下角
P1 = [x2, y2, 0];  % 右下角
% 顶边两个点（向Z轴方向抬升）
P2 = P1 + [0, 0, H_real];  % 右上角
P3 = P0 + [0, 0, H_real];  % 左上角

% 5. 构建面片顶点和贴图
X = [P0(1), P1(1); P3(1), P2(1)];
Y = [P0(2), P1(2); P3(2), P2(2)];
Z = [P0(3), P1(3); P3(3), P2(3)];

% 6. 在 3D 中贴图显示
figure;
surf(X, Y, Z, flipud(img_crop), ...
    'FaceColor', 'texturemap', ...
    'EdgeColor', 'none');

xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal;
axis([0 25 0 25 0 25]);  % 设置所有轴范围为 0–25
view(3); grid on;
title('沿底边方向自动构建的障碍板');
