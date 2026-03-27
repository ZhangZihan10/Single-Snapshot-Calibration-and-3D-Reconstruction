%%失败

function [C_B, R_WC, T_BC, diagInfo] = pose_from_board_no_rotation(calib_data, kk, squareSize, doPlot)
% 以棋盘原点为世界原点(=B系)，估计相机在棋盘系中的位姿
% 输入
%   calib_data.ocam_model : OCam 标定模型
%   calib_data.n_sq_x/y   : 棋盘内格数
%   calib_data.L{kk}      : 图像路径
%   squareSize            : 单格边长(米)
%   doPlot (可选)         : 是否绘图，默认 true
% 输出
%   C_B   : 3x1 相机在棋盘系 B 的位置
%   R_WC  : 3x3 相机在棋盘系 B 的姿态(estimateWorldCameraPose 的 Orientation)
%   T_BC  : 4x4 变换 ^B T_C
%   diagInfo: 诊断信息(轴配置、内点索引、RMSE)

if nargin < 4, doPlot = true; end

% ---------- 0) 角点（像素） ----------
need_detect = true;
if isfield(calib_data,'Xp_abs') && ndims(calib_data.Xp_abs)>=3 ...
   && size(calib_data.Xp_abs,3) >= kk && ~isempty(calib_data.Xp_abs(:,:,kk))
    need_detect = false;
end

if need_detect
    use_corner_find = true;
    
        [cb, Xp, Yp] = get_checkerboard_cornersUrban(kk, use_corner_find, calib_data);
    
    if cb~=1, error('角点检测失败。'); end
    calib_data.Xp_abs(:,:,kk) = Xp;   % 行(y)
    calib_data.Yp_abs(:,:,kk) = Yp;   % 列(x)
end

Xp = calib_data.Xp_abs(:,:,kk);  Xp = Xp(:);     % 行
Yp = calib_data.Yp_abs(:,:,kk);  Yp = Yp(:);     % 列
imgPts_pix = [Yp, Xp];                            % [u v]

% ---------- 1) 自动匹配棋盘世界点的轴向(32 组组合里选 RMSE 最小) ----------
nx = calib_data.n_sq_x + 1; 
ny = calib_data.n_sq_y + 1;
cfgs = dec2bin(0:31)-'0';   % [origin_right origin_bottom swapXY flipX flipY]
best.rmse = inf;

% 先把像素 -> 单位光线 -> 归一化坐标（只做一次）
rays0 = cam2world_ocam_batch(imgPts_pix, calib_data.ocam_model);
if mean(rays0(:,3)) < 0, rays0 = -rays0; end
normPts0 = [rays0(:,1)./rays0(:,3), rays0(:,2)./rays0(:,3)];
Kvirt = [1000 0 0; 0 1000 0; 0 0 1];
imgVirt0 = [normPts0(:,1)*Kvirt(1,1), normPts0(:,2)*Kvirt(2,2)];
camParams = cameraParameters('IntrinsicMatrix',Kvirt');

for r = 1:size(cfgs,1)
    origin_right = logical(cfgs(r,1));
    origin_bottom= logical(cfgs(r,2));
    swap_xy      = logical(cfgs(r,3));
    flip_x       = logical(cfgs(r,4));
    flip_y       = logical(cfgs(r,5));

    [ii, jj] = ndgrid(0:nx-1, 0:ny-1);
    if origin_right,  ii = (nx-1) - ii; end
    if origin_bottom, jj = (ny-1) - jj; end

    if swap_xy
        Xw = jj(:) * squareSize;
        Yw = ii(:) * squareSize;
    else
        Xw = ii(:) * squareSize;
        Yw = jj(:) * squareSize;
    end
    if flip_x, Xw = -Xw; end
    if flip_y, Yw = -Yw; end

    wp = [Xw, Yw, zeros(nx*ny,1)];

    % 用平面单应近似在归一化平面上计算 RMSE 作为一致性指标
    try
        [Rcw, tcw] = extrinsics(imgVirt0, wp(:,1:2), camParams); % world->camera
        H = [Rcw(:,1:2), tcw(:)];                % 3x3
        proj = (H * [wp(:,1:2), ones(nx*ny,1)].')';
        proj = proj(:,1:2) ./ proj(:,3);
        rmse = sqrt(mean(sum((proj - normPts0).^2,2)));
    catch
        rmse = inf;
    end

    if rmse < best.rmse
        best.rmse = rmse;
        best.cfg  = [origin_right origin_bottom swap_xy flip_x flip_y];
        best.wp   = wp;
    end
end

worldPoints_B = best.wp;

% ---------- 2) 重新用最优轴向做姿态求解 ----------
% （已经有 imgVirt0 / camParams / normPts0）
[ R_WC, C_B, inlierIdx, status ] = estimateWorldCameraPose( ...
    imgVirt0, worldPoints_B, camParams, ...
    'MaxReprojectionError', 5, 'Confidence', 99, 'MaxNumTrials', 5000);

if status~=0
    warning('estimateWorldCameraPose status=%d，结果可能不够稳。', status);
end

% 4x4
T_BC = eye(4);
T_BC(1:3,1:3) = R_WC;
T_BC(1:3,4)   = C_B(:);

% ---------- 3) 诊断信息 ----------
diagInfo.cfg   = best.cfg;     % [right bottom swapXY flipX flipY]
diagInfo.rmse  = best.rmse;
diagInfo.inliers = inlierIdx;
diagInfo.note  = 'Origin at board corner chosen by cfg; Z_B=0 plane';

% ---------- 4) 可视化 ----------
if doPlot
    figure('Name','Camera pose in Board frame'); hold on; grid on; axis equal; view(35,25);
    % 棋盘网格
    draw_board_mesh(worldPoints_B, nx, ny);
    % 坐标轴
    draw_frame(eye(4), 0.5*squareSize, 'B');
    % 相机
    plotCamera('Location',C_B','Orientation',R_WC,'Size',0.8*squareSize,'Color','r','Opacity',0.2);
    xlabel('X_B'); ylabel('Y_B'); zlabel('Z_B');
    title(sprintf('Image %d | cfg=[%d %d %d %d %d]  rmse=%.4f', kk, best.cfg, best.rmse));
end
end

% ===== 工具函数 =====
function rays = cam2world_ocam_batch(uv, ocam)
% uv: N×2 [u v]; 输出 N×3 单位光线
N = size(uv,1); rays = zeros(N,3);
for k=1:N
    rays(k,:) = cam2world([uv(k,1); uv(k,2)], ocam);
end
rays = rays ./ vecnorm(rays,2,2);
end

function draw_frame(T, s, tag)
O=T(1:3,4); X=O+s*T(1:3,1); Y=O+s*T(1:3,2); Z=O+s*T(1:3,3);
plot3([O(1) X(1)],[O(2) X(2)],[O(3) X(3)],'r-','LineWidth',2); text(X(1),X(2),X(3),[tag,'_X'],'Color','r');
plot3([O(1) Y(1)],[O(2) Y(2)],[O(3) Y(3)],'g-','LineWidth',2); text(Y(1),Y(2),Y(3),[tag,'_Y'],'Color','g');
plot3([O(1) Z(1)],[O(2) Z(2)],[O(3) Z(3)],'b-','LineWidth',2); text(Z(1),Z(2),Z(3),[tag,'_Z'],'Color','b');
plot3(O(1),O(2),O(3),'ko','MarkerFaceColor','y'); text(O(1),O(2),O(3),['  ',tag,'_O'],'Color','k');
end

function draw_board_mesh(wp, nx, ny)
% wp: (nx*ny)×3，按列优先/行优先均可，下面用网格重整
X = reshape(wp(:,1), nx, ny);
Y = reshape(wp(:,2), nx, ny);
Z = zeros(size(X));
mesh(X,Y,Z,'EdgeColor',[0.4 0.4 0.4],'FaceColor','none');
end
