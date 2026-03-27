%%成功     真实环境

function [C_B, R_CW, T_BC, dbg] = pose_from_board_no_rotation7(calib_data, kk, squareSize, use_corner_find,opts)
% 以“角度误差 RANSAC + 鱼眼模型精修”的方式计算相机位姿
% 输出：
%   C_B   : 1x3，相机在棋盘(=世界)中的位置
%   R_CW  : 3x3，world->camera 旋转
%   T_BC  : 4x4，^B T_C 齐次矩阵
%   dbg   : 结构体（rmse_px/angle_deg/inliers等）

if nargin < 5 || isempty(opts), opts = struct; end
opts = set_default(opts, 'angThreshDeg', 1.0);     % RANSAC角度阈值（单位°）
opts = set_default(opts, 'maxTrials',   3000);     % RANSAC迭代上限
opts = set_default(opts, 'confidence',  0.999);    % 目标置信度
opts = set_default(opts, 'refine',      'fisheye');% 'fisheye' 或 'bearing'
opts = set_default(opts, 'verbose',     true);

% ---------- 1) 角点 ----------
[cb, Xp, Yp] = get_checkerboard_cornersUrban9(kk, use_corner_find, calib_data);
assert(cb==1, 'Corner detection failed.');
Xp = Xp(:); Yp = Yp(:);
imgPts = [Yp, Xp];               % [u,v]=[列,行]

% ---------- 2) 棋盘3D ----------
n_sq_x = calib_data.n_sq_x; n_sq_y = calib_data.n_sq_y;
[Xg, Yg] = meshgrid(0:n_sq_x, 0:n_sq_y);
WP = [Xg(:)*squareSize, Yg(:)*squareSize, zeros(numel(Xg),1)];
N  = min(size(WP,1), size(imgPts,1));
WP = WP(1:N,:); imgPts = imgPts(1:N,:);

% ---------- 3) 像素 -> 相机系单位光线 ----------
rays = cam2world_ocam_batch(imgPts, calib_data.ocam_model); % N×3
rays = normalize_rows(rays);
if mean(rays(:,3))<0, rays = -rays; end

% ---------- 4) RANSAC (角度误差判内点) ----------
% 为了拿到候选姿态，这里用 unit-K 的 PnP（内置），但内点评估用角度误差
[bestR, bestC, inliers] = ransac_angle_pnp(rays, WP, opts);

% ---------- 5) 非线性精修 ----------
switch lower(opts.refine)
    case 'fisheye'
        has_fisheye = (exist('world2cam','file')==2);
        if has_fisheye
            [R_CW, C_B, rmse_px] = refine_angle(bestR, bestC, WP(inliers,:), rays(inliers,:));
            %[R_CW, C_B, rmse_px] = refine_fisheye(bestR, bestC, WP(inliers,:), imgPts(inliers,:), calib_data.ocam_model);
        else
            warning('world2cam not found; fallback to bearing-angle refinement.');
            [R_CW, C_B, rmse_px] = refine_angle(bestR, bestC, WP(inliers,:), rays(inliers,:));
        end
    otherwise
        [R_CW, C_B, rmse_px] = refine_angle(bestR, bestC, WP(inliers,:), rays(inliers,:));
end

% ---------- 6) 打包输出 ----------
T_BC = eye(4); T_BC(1:3,1:3)=R_CW; T_BC(1:3,4)=C_B(:);
% 角度RMSE（度）
ang_err = bearing_rmse_deg(R_CW, C_B, WP, rays);

% ===== 计算相机最终旋转角（在可视化之前粘贴） =====
R_WC = R_CW.';   % camera -> world

% 世界系ZYX欧拉角：R_WC = Rz(yaw)*Ry(pitch)*Rx(roll)
yaw   = atan2( R_WC(2,1), R_WC(1,1) );
pitch = asin( -R_WC(3,1) );
roll  = atan2( R_WC(3,2), R_WC(3,3) );

yaw_deg   = rad2deg(yaw);
pitch_deg = rad2deg(pitch);
roll_deg  = rad2deg(roll);

% 相机光轴(+Zc)在棋盘平面上的投影与方位
zc_world = R_WC(:,3); 
%fprintf('zc_world = [%.3f %.3f %.3f]\n', zc_world);% 相机Z轴在世界系
if zc_world(3) > 0
    % ---- 光轴朝上，需要翻转 ----
    % 1. 翻转旋转矩阵：绕世界Z轴旋转180°
    Rflip = [ -1  0  0;
               0 -1  0;
               0  0  1 ];
    R_CW = Rflip * R_CW;   % 更新 world->camera

    % 2. 相机位置对称到棋盘另一侧 (Z轴取负)
    C_B(3) = -C_B(3);
end



zproj    = zc_world - dot(zc_world,[0;0;1])*[0;0;1];   % 投影到Z_B=0
if norm(zproj) > 1e-12
    zproj = zproj / norm(zproj);
    heading_deg = atan2d(zproj(2), zproj(1));          % 从+X_B逆时针为正
    theta_x_deg = acosd( max(-1,min(1, dot(zproj,[1;0;0]) )) );  % ∠(投影,+X_B)
    theta_y_deg = acosd( max(-1,min(1, dot(zproj,[0;1;0]) )) );  % ∠(投影,+Y_B)
else
    heading_deg = NaN; theta_x_deg = NaN; theta_y_deg = NaN;
end

% 控制台输出（可选）
disp('=== Final camera orientation (camera->world) ===');
disp('R_WC ='); disp(R_WC);
fprintf('Euler ZYX (world): roll_x=%.3f°, pitch_y=%.3f°, yaw_z=%.3f°\n', ...
        roll_deg, pitch_deg, yaw_deg);
fprintf('Optical axis heading on board XY: %.3f°  (∠X_B=%.3f°, ∠Y_B=%.3f°)\n', ...
        heading_deg, theta_x_deg, theta_y_deg);

% 存入调试结构
dbg.R_WC                  = R_WC;
dbg.euler_zyx_world_deg   = [roll_deg, pitch_deg, yaw_deg];   % [roll, pitch, yaw]
dbg.opt_axis_heading_deg  = heading_deg;
dbg.opt_axis_to_XB_deg    = theta_x_deg;
dbg.opt_axis_to_YB_deg    = theta_y_deg;


% 可视化（可选）
if opts.verbose
    I = imread(calib_data.L{kk});
    figure(3); clf; imshow(I,[]); title(sprintf('Image %d',kk)); hold on;
    plot(imgPts(:,1), imgPts(:,2), 'r+'); hold off;

    figure(4); clf; hold on; grid on; axis equal; view(40,25);
    BX = reshape(WP(:,1), n_sq_y+1, n_sq_x+1);
    BY = reshape(WP(:,2), n_sq_y+1, n_sq_x+1);
    surf(BX, BY, zeros(size(BX)), 'FaceColor',[.9 .9 .9], 'EdgeColor',[.6 .6 .6]);
    local_draw_frame(eye(4), squareSize*0.25, 'B');
    plotCamera('Location', C_B, 'Orientation', R_CW', 'Size', squareSize*1.2, 'Color','r');
    title(sprintf('Image %d | inliers=%d/%d | rmse_px=%.3g | rmse_ang=%.3g°', ...
        kk, numel(inliers), N, rmse_px, ang_err));
    xlabel X_B; ylabel Y_B; zlabel Z_B; hold off;
end

% dbg
dbg.inliers = inliers; dbg.rmse_px = rmse_px; dbg.rmse_ang_deg = ang_err;
dbg.C_B = C_B; dbg.R_CW = R_CW;

end

% ===================== 辅助函数们 =====================

function [R_CW, C_B, inliers] = ransac_angle_pnp(rays, WP, opts)
N = size(WP,1);
angThr = deg2rad(opts.angThreshDeg);
bestScore = -inf; inliers = 1:N;

% 预生成单位内参下的“虚拟像素”（直接用归一化坐标）
xn = rays(:,1)./rays(:,3); yn = rays(:,2)./rays(:,3);
imgNorm = [xn, yn];
camParams = cameraParameters('IntrinsicMatrix', eye(3)); % K=I

s = 4; % 子集大小
maxTrials = opts.maxTrials;
% 自适应终止（简单实现）
for t=1:maxTrials
    idx = randsample(N, s, false);
    try
        [R_CW_cand, C_cand] = pnp_subset(imgNorm(idx,:), WP(idx,:), camParams);
    catch
        continue
    end
    ang = angular_errors(R_CW_cand, C_cand, WP, rays);   % 弧度
    inl = find(ang <= angThr);
    score = numel(inl);

    if score > bestScore
        bestScore = score;
        inliers = inl;
        R_CW = R_CW_cand;
        C_B  = C_cand(:).';
        % （可选）根据内点率自适应减少迭代
        w = score / N;
        if w > 0
            s_hat = s;
            p = opts.confidence;
            maxTrials = min(maxTrials, ceil(log(1-p)/log(max(1e-12,1-w^s_hat))));
        end
    end
end


% 内点上再做一次 EPnP 得到更稳的初值
[R_CW, C_B] = pnp_subset(imgNorm(inliers,:), WP(inliers,:), camParams);
C_B = C_B(:).';
end

function ang = angular_errors(R_CW, C_B, WP, rays)
% 计算每个点的光线夹角误差（弧度）
Xc = (R_CW*(WP.' - C_B(:))).';          % N×3
Xc = normalize_rows(Xc);
ang = real(acos( max(-1, min(1, sum(Xc.*rays,2))) ));
end

function rmse_deg = bearing_rmse_deg(R_CW, C_B, WP, rays)
ang = angular_errors(R_CW, C_B, WP, rays);
rmse_deg = rad2deg( sqrt(mean(ang.^2)) );
end

function [R_CW, C_B] = pnp_subset(imgNorm, WP, camParams)
% 用 K=I 的内参调用 MATLAB 的 EPnP/最小子集求解
[R_CW, camLoc] = estimateWorldCameraPose(imgNorm, WP, camParams, ...
    'MaxReprojectionError', 2.0, 'Confidence', 99.9, 'MaxNumTrials', 500);
C_B = camLoc(:);
end

function [R_CW, C_B, rmse_px] = refine_fisheye(R0, C0, WP, uv, ocam)
% 基于 world2cam 的像素重投影误差精修
x0 = [rotvec_from_R(R0); C0(:)];
fun = @(x) reproj_residual_pix(x, WP, uv, ocam);
x_opt = local_least_squares(fun, x0);
[R_CW, C_B] = unpack_pose(x_opt);
% 计算RMSE（像素）
r = fun(x_opt);
rmse_px = sqrt(mean(reshape(r,2,[]).^2,'all'));
end

function [R_CW, C_B, rmse_px] = refine_angle(R0, C0, WP, rays)
% 最小化光线夹角（弧度）
x0 = [rotvec_from_R(R0); C0(:)];
fun = @(x) angle_residual_rad(x, WP, rays);
x_opt = local_least_squares(fun, x0);
[R_CW, C_B] = unpack_pose(x_opt);
ang = reshape(fun(x_opt),1,[]);
rmse_px = NaN; % 无像素尺度
end

% ---------- 残差函数 ----------
function r = reproj_residual_pix(x, WP, uv, ocam)
[R, C] = unpack_pose(x);
Pc = (R*(WP.' - C)).';   % N×3
N = size(Pc,1);
pred = zeros(N,2);
for i=1:N
    pred(i,:) = world2cam(Pc(i,:).', ocam).';
end
r = reshape((pred - uv).', [], 1);  % 2N×1
end

function r = angle_residual_rad(x, WP, rays)
[R, C] = unpack_pose(x);
Xc = (R*(WP.' - C)).';
Xc = normalize_rows(Xc);
dotv = sum(Xc.*rays,2);
dotv = max(-1, min(1, dotv));
r = acos(dotv);                           % N×1 (弧度)
end

% ---------- 优化器（lsqnonlin优先，缺省退化到fminsearch） ----------
function x_opt = local_least_squares(fun, x0)
if exist('lsqnonlin','file')==2
    opts = optimoptions('lsqnonlin','Display','off','MaxIter',200,'FunctionTolerance',1e-10);
    x_opt = lsqnonlin(fun, x0, [], [], opts);
else
    % 简单退化（不如lsqnonlin稳，但可用）
    obj = @(x) sum(fun(x).^2);
    x_opt = fminsearch(obj, x0);
end
end

% ---------- 姿态打包/解包 ----------
function w = rotvec_from_R(R)
th = acos( max(-1,min(1,(trace(R)-1)/2)) );
if th < 1e-12
    w = [0;0;0];
else
    wx = (R - R.')/(2*sin(th));
    w = th * [wx(3,2); wx(1,3); wx(2,1)];
end
end

function [R, C] = unpack_pose(x)
w = x(1:3); C = x(4:6);
th = norm(w);
if th < 1e-12
    R = eye(3);
else
    k = w / th;
    K = [   0    -k(3)  k(2);
          k(3)    0    -k(1);
         -k(2)  k(1)    0  ];
    R = eye(3) + sin(th)*K + (1-cos(th))*(K*K);
end
end

% ---------- 杂项 ----------
function rays = cam2world_ocam_batch(uv, ocam)
N = size(uv,1); rays = zeros(N,3);
for k=1:N
    % cam2world 的输入是 [row; col] = [v; u]
    rays(k,:) = cam2world([uv(k,2); uv(k,1)], ocam).';
end
rays = normalize_rows(rays);
% 可保留这句以确保投影到 pinhole 时 z>0
%if mean(rays(:,3)) < 0, rays = -rays; end
end

function A = normalize_rows(A)
n = vecnorm(A,2,2); n(n==0)=1; A = A ./ n;
end

function local_draw_frame(T, s, tag)
O=T(1:3,4); X=O+s*T(1:3,1); Y=O+s*T(1:3,2); Z=O+s*T(1:3,3);
plot3([O(1) X(1)],[O(2) X(2)],[O(3) X(3)],'r-','LineWidth',2); text(X(1),X(2),X(3),[tag,'_X'],'Color','r');
plot3([O(1) Y(1)],[O(2) Y(2)],[O(3) Y(3)],'g-','LineWidth',2); text(Y(1),Y(2),Y(3),[tag,'_Y'],'Color','g');
plot3([O(1) Z(1)],[O(2) Z(2)],[O(3) Z(3)],'b-','LineWidth',2); text(Z(1),Z(2),Z(3),[tag,'_Z'],'Color','b');
plot3(O(1),O(2),O(3),'ko','MarkerFaceColor','y');
end

function s = set_default(s, name, val)
if ~isfield(s,name) || isempty(s.(name)), s.(name) = val; end
end