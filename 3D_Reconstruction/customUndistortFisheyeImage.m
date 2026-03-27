function undistorted = customUndistortFisheyeImage(fisheyeImg, cameraParams, outputSize, viewAngles)

% 解析输入
[H_out, W_out] = deal(outputSize(1), outputSize(2));
intrinsics = cameraParams.Intrinsics;
fisheyeImg = im2double(fisheyeImg);
[H_in, W_in, ~] = size(fisheyeImg);

% 构建输出图像像素网格
[xx, yy] = meshgrid(1:W_out, 1:H_out);

% 视场角（可调，这里设为 90°）
fov_deg = 90;
fov = deg2rad(fov_deg);
aspect = W_out / H_out;
plane_w = 2 * tan(fov/2);
plane_h = plane_w / aspect;

% 归一化到[-1,1]
nx = (2*(xx - 0.5)/W_out - 1) * plane_w/2;
ny = (1 - 2*(yy - 0.5)/H_out) * plane_h/2;
nz = ones(size(nx));
rays = cat(3, nx, ny, nz);

% 转换为单位向量
rays = reshape(rays, [], 3);
rays = normalizeRows(rays);

% 应用旋转
R = makeRotationMatrix(deg2rad(viewAngles));
rays = (R * rays')';

% 转换为球坐标
[theta, phi, ~] = cart2sph(rays(:,1), rays(:,2), rays(:,3));
r = (pi/2 - phi);  % 极角作为半径（单位：弧度）

% 从模型获取映射函数
distort_r = r / deg2rad(180);  % 线性鱼眼映射比例
u = intrinsics.DistortionCenter(1) + distort_r .* cos(theta) * intrinsics.ImageSize(2)/2;
v = intrinsics.DistortionCenter(2) + distort_r .* sin(theta) * intrinsics.ImageSize(1)/2;

% 限制范围
valid = u >= 1 & u <= W_in & v >= 1 & v <= H_in;

% 图像重采样
undistorted = zeros(H_out, W_out, 3);
for c = 1:3
    channel = interp2(fisheyeImg(:,:,c), u(valid), v(valid), 'linear', 0);
    temp = zeros(H_out*W_out, 1);
    temp(valid) = channel;
    undistorted(:,:,c) = reshape(temp, H_out, W_out);
end
end

% 辅助函数：旋转矩阵
function R = makeRotationMatrix(angles)
    Rx = [1 0 0; 0 cos(angles(1)) -sin(angles(1)); 0 sin(angles(1)) cos(angles(1))];
    Ry = [cos(angles(2)) 0 sin(angles(2)); 0 1 0; -sin(angles(2)) 0 cos(angles(2))];
    Rz = [cos(angles(3)) -sin(angles(3)) 0; sin(angles(3)) cos(angles(3)) 0; 0 0 1];
    R = Rz * Ry * Rx;
end

% 单位化向量
function V = normalizeRows(M)
    V = M ./ vecnorm(M, 2, 2);
end
