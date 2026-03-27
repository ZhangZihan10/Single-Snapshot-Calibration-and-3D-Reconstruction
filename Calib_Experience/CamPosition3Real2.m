
%% 真实 单张校准相机，激光面，定位    先转换为透视相机再计算姿态
%% =======================================================
%  路线 B：鱼眼像素去畸变 → 针孔像素 → PnP 姿态估计
%  输出相机欧拉角 (roll, pitch, yaw) 与世界坐标位置
% =======================================================
%% 单张姿态：鱼眼 → 针孔 → PnP（正确重投影 & 精简版）
clc; clear; close all;

%% 1) 加载参数与图像
load('fisheyeCameraParamsReal1.mat');   % 提供 cameraParams (fisheyeParameters)
%I = imread('c1.jpg');
I = imread('testr13.jpg');

intrinsicsIn = cameraParams.Intrinsics; % fisheyeIntrinsics
szI   = [size(I,1) size(I,2)];          % [H W]
szInt = intrinsicsIn.ImageSize;         % 标定图像尺寸 [H W]

% 若图像宽高被交换，仅旋转一次修正；否则不随意缩放图像
if ~isequal(szI, szInt) && isequal(szI, fliplr(szInt))
    I = rot90(I);
    szI = [size(I,1) size(I,2)];
end

% 若尺寸确实不同（纯缩放场景），按比例更新内参，而非缩放图
if ~isequal(szI, intrinsicsIn.ImageSize)
    sx = szI(2)/szInt(2); sy = szI(1)/szInt(1);
    newF = intrinsicsIn.FocalLength    .* [sx sy];
    newC = intrinsicsIn.PrincipalPoint .* [sx sy];
    try
        intrinsicsIn = fisheyeIntrinsics(newF, newC, szI, intrinsicsIn.MappingCoefficients);
    catch
        intrinsicsIn = fisheyeIntrinsics(newF, newC, szI);
    end
end

%% 2) 棋盘角点（在鱼眼图上检测）
[imagePoints, boardSize] = detectCheckerboardPoints(I);
assert(~isempty(imagePoints), '未检测到棋盘角点');

% 在原图上显示检测结果
figure; imshow(I); hold on;
plot(imagePoints(:,1), imagePoints(:,2), 'go', 'MarkerSize',6, 'LineWidth',1.5);

% 可选：给每个角点编号，便于检查
for k = 1:size(imagePoints,1)
    text(imagePoints(k,1)+5, imagePoints(k,2), num2str(k), ...
         'Color','y','FontSize',8,'FontWeight','bold');
end

title(sprintf('Detected Checkerboard (%d×%d) Corners', boardSize(1)-1, boardSize(2)-1));
legend('检测到的角点');
hold off;
squareSize   = 26; % mm
worldPoints2d = generateCheckerboardPoints(boardSize, squareSize);
worldPoints3d = [worldPoints2d, zeros(size(worldPoints2d,1),1)];   % Z=0 平面

%% 3) 鱼眼→针孔（点与图像一致：OutputView='same' & ScaleFactor=1.0）
undistortedPts = undistortFisheyePoints(imagePoints, intrinsicsIn);
[J, intrinsicsOut] = undistortFisheyeImage(I, intrinsicsIn, ...
    'OutputView','same', 'ScaleFactor', 1.0);

%% 4) PnP（针孔模型）
[rotMat, transVec, inlierIdx] = estimateWorldCameraPose( ...
    undistortedPts, worldPoints3d, intrinsicsOut, ...
    'MaxReprojectionError',2.0,'Confidence',99.99,'MaxNumTrials',3000);

fprintf('Inliers: %d / %d\n', nnz(inlierIdx), size(worldPoints3d,1));

%% 5) 外参 → 欧拉角与相机中心（世界系）
R_WC = rotMat.';                 % camera -> world
C_W  = -R_WC * transVec.';       % 相机中心（世界系, 列向量）

eulZYX = rotm2eul(rotMat);       % [yaw pitch roll] (rad)  world->camera, ZYX
yaw_deg   = rad2deg(eulZYX(1));
pitch_deg = rad2deg(eulZYX(2));
roll_deg  = rad2deg(eulZYX(3));

fprintf('C_W = [%.2f %.2f %.2f] mm | roll=%.2f  pitch=%.2f  yaw=%.2f (deg)\n', ...
    C_W(1), C_W(2), C_W(3), roll_deg, pitch_deg, yaw_deg);

%% （可选）6) 关于 z=0 平面镜像相机姿态（默认关闭）
DO_MIRROR_Z = false;
if DO_MIRROR_Z
    S     = diag([1 1 -1]);
    R_WC  = S * R_WC * S;           % camera->world（镜像）
    C_W   = S * C_W;                % 位置镜像
    rotMat   = R_WC.';              % world->camera
    transVec = -rotMat * C_W;       % 由相机中心回推 t
end

%% 7) 重投影验证（在去畸变图 J 上）
reprojPts = worldToImage(intrinsicsOut, rotMat, transVec, worldPoints3d);
pxErr = sqrt(sum((reprojPts - undistortedPts).^2, 2));
fprintf('Reprojection error: mean=%.2f  med=%.2f  p95=%.2f  max=%.2f px\n', ...
    mean(pxErr), median(pxErr), prctile(pxErr,95), max(pxErr));

figure; imshow(J); hold on;
plot(undistortedPts(:,1), undistortedPts(:,2), 'go', 'MarkerSize',5, 'LineWidth',1);
plot(reprojPts(:,1),      reprojPts(:,2),      'r+', 'MarkerSize',5, 'LineWidth',1);
legend({'去畸变角点','重投影点'}); title('重投影验证（针孔域）'); hold off;

%% 8) 3D 可视化（相机与棋盘）
figure; hold on; grid on; axis equal; view(35,20);
xlabel X; ylabel Y; zlabel Z;
plot3(worldPoints3d(:,1), worldPoints3d(:,2), worldPoints3d(:,3), 'k.', 'MarkerSize',8);
plotCamera('Location', C_W.', 'Orientation', R_WC, 'Size', 50, 'Color', 'r', 'Opacity', 0.2);
legend({'棋盘角点','相机'}); title('Camera pose (world frame)');
