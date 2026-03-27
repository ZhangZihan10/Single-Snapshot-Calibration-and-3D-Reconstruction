





function Fish2normal3(input_img, output_img, varargin)
% FISHEYE2PERSP - 鱼眼图像转透视投影
% 参数说明：
%   input_img   输入鱼眼图像路径
%   output_img  输出透视图像路径
% 可选参数：
%   'Width'      输出宽度(默认800)
%   'Height'     输出高度(默认600)
%   'PerspFOV'   透视视场角(默认100度)
%   'FishFOV'    鱼眼视场角(默认180度)
%   'Center'     鱼眼中心坐标[x,y](默认图像中心)
%   'Radius'     鱼眼半径[rx,ry](默认图像短边一半)
%   'Rotations'   旋转角度[pan,tilt,roll](默认[0,0,0])
%   'AntiAlias'  抗锯齿级别(默认2)
%   'Distortion' 畸变校正系数[a1,a2,a3,a4](默认不校正)


cameraParams = [];
if nargin >= 3 && isa(varargin{1}, 'fisheyeParameters')
    cameraParams = varargin{1};
    varargin(1) = [];
end
% 解析输入参数
p = inputParser;
p.addParameter('Width', 1920);
p.addParameter('Height', 1920);
p.addParameter('PerspFOV', 100);
p.addParameter('FishFOV', 180);
p.addParameter('Center', []);
p.addParameter('Radius', []);
p.addParameter('Rotations', [0,0,0]);
p.addParameter('AntiAlias', 2);
p.addParameter('Distortion', []);
p.parse(varargin{:});

% 如果传入了 cameraParams，用它覆盖相关变量（而不是p.Results）
if ~isempty(cameraParams)
    imgSize = cameraParams.Intrinsics.ImageSize;
    fish_center = cameraParams.Intrinsics.DistortionCenter;
    radius_val = [min(imgSize)/2, min(imgSize)/2];
    distCoeffs = cameraParams.Intrinsics.MappingCoefficients;
    
    % 视场角如果用户没传，或者是默认180度，就用默认的180，否则保持用户传入
    %if isempty(FishFOV) || FishFOV == 180
    %    FishFOV = 180;
    %end
    
    % 覆盖局部变量
    Center = fish_center;
    Radius = radius_val;
    Distortion = distCoeffs;
end



% 参数初始化
persp_size = [p.Results.Height, p.Results.Width];
fov_persp = deg2rad(p.Results.PerspFOV);
fov_fish = deg2rad(p.Results.FishFOV);
rot_angles = deg2rad(p.Results.Rotations);
aa_level = p.Results.AntiAlias;
dist_coeffs = Distortion;

% 读取鱼眼图像
fish_img = im2double(imread(input_img));
[fish_h, fish_w, ~] = size(fish_img);

% 设置默认鱼眼中心
if isempty(Center)
    fish_center = [fish_w/2, fish_h/2];
else
    fish_center = Center;
end

% 设置默认鱼眼半径
if isempty(Radius)
    fish_radius = min(fish_center)/2 * [1, 1];
else
    fish_radius = Radius;
end

% 创建透视图像网格
[xx, yy] = meshgrid(1:persp_size(2), 1:persp_size(1));

% 抗锯齿子采样
if aa_level > 1
    sub_x = linspace(0, 1-1/aa_level, aa_level);
    sub_y = linspace(0, 1-1/aa_level, aa_level);
    [sx, sy] = meshgrid(sub_x, sub_y);
else
    sx = 0; sy = 0;
end

% 计算透视投影平面参数
aspect = fish_radius(2)/fish_radius(1);
tan_half_fov = tan(fov_persp/2);
persp_plane = struct(...
    'w', 2*tan_half_fov, ...      % 投影平面宽度
    'h', 2*tan_half_fov*persp_size(1)/persp_size(2),... % 高度
    'dist', 1 ...                % 投影平面距离
);

% 生成旋转矩阵 (Z-Y-X顺序)
R = makeRotationMatrix(rot_angles);

% 主处理循环
persp_h = p.Results.Height;
persp_w = p.Results.Width;
reshape_size = [persp_h, persp_w];
persp_img = zeros(persp_h, persp_w, 3);

for i = 1:aa_level
    for j = 1:aa_level
        % 计算子采样偏移
        x = xx + sx(i,j);
        y = yy + sy(i,j);

        % 归一化坐标到[-1,1]
        nx = (2*(x-1)/(persp_w-1) - 1) * persp_plane.w/2;
        ny = (1 - 2*(y-1)/(persp_h-1)) * persp_plane.h/2;

        % 射线方向向量
        ray_dir = [nx(:), ny(:), ones(numel(nx),1)*persp_plane.dist];
        ray_dir = ray_dir ./ vecnorm(ray_dir, 2, 2);

        % 应用旋转
        ray_dir = (R * ray_dir')';

        % 球坐标转换
        [az, el, ~] = cart2sph(ray_dir(:,1), ray_dir(:,3), ray_dir(:,2));
        theta = az;
        phi = pi/2 - el;

        % 鱼眼半径映射
        if ~isempty(dist_coeffs)
            r = phi .* polyval(dist_coeffs, phi);
        else
            r = phi / fov_fish;
        end

        % 图像坐标
        u = fish_center(1) + fish_radius(1)*r.*cos(theta);
        v = fish_center(2) + fish_radius(2)*r.*sin(theta);

        % 插值
        for ch = 1:3
            interp_val = interp2(fish_img(:,:,ch), u, v, 'linear', 0);
            persp_img(:,:,ch) = persp_img(:,:,ch) + reshape(interp_val, reshape_size);
        end
    end
end

% 归一化并保存结果
persp_img = persp_img / aa_level^2;
imwrite(persp_img, output_img);

figure;
subplot(1,2,1); imshow(input_img); title('原始图像');
subplot(1,2,2); imshow(output_img); title('校正后图像');


end



function R = makeRotationMatrix(angles)
% 创建组合旋转矩阵 (Z-Y-X顺序)
% angles: [pan(z), tilt(x), roll(y)]

Rx = [1 0 0; 
      0 cos(angles(2)) -sin(angles(2));
      0 sin(angles(2)) cos(angles(2))];
  
Ry = [cos(angles(3)) 0 sin(angles(3));
       0 1 0;
       -sin(angles(3)) 0 cos(angles(3))];
  
Rz = [cos(angles(1)) -sin(angles(1)) 0;
       sin(angles(1)) cos(angles(1)) 0;
       0 0 1];
   
R = Rz * Ry * Rx;
end

