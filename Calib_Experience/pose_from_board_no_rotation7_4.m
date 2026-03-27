%%测试 使用像素误差版 RANSAC PnP  虚拟环境与真实环境   加坐标轴

function [C_B, R_CW, T_BC, dbg,roll_deg, pitch_deg, yaw_deg] = pose_from_board_no_rotation7_4(calib_data, kk, squareSize, use_corner_find,opts)
% 以“角度误差 RANSAC + 鱼眼模型精修”的方式计算相机位姿
% 输出：
%   C_B   : 1x3，相机在棋盘(=世界)中的位置
%   R_CW  : 3x3，world->camera 旋转
%   T_BC  : 4x4，^B T_C 齐次矩阵
%   dbg   : 结构体（rmse_px/angle_deg/inliers等）

if nargin < 5 || isempty(opts), opts = struct; end
opts = set_default(opts, 'angThreshDeg', 0.5);     % RANSAC角度阈值（单位°）
opts = set_default(opts, 'maxTrials',   3000);     % RANSAC迭代上限
opts = set_default(opts, 'confidence',  0.999);    % 目标置信度
opts = set_default(opts, 'refine',      'fisheye');% 'fisheye' 或 'bearing'
opts = set_default(opts, 'verbose',     true);
opts = set_default(opts, 'pixThreshPx', 3.0);  


% ---------- 1) 角点 ----------
[cb, Xp, Yp] = get_checkerboard_cornersUrban9(kk, use_corner_find, calib_data);
assert(cb==1, 'Corner detection failed.');
%Xp = Xp(:); Yp = Yp(:);
Xp = Xp(:); Yp = Yp(:);
imgPts = [Yp, Xp];               % [u,v]=[列,行]

% ---------- 2) 棋盘3D (交换X/Y定义，使短边为Y轴，长边为X轴) ----------
n_sq_x = calib_data.n_sq_x;
n_sq_y = calib_data.n_sq_y;
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
%[bestR, bestC, inliers] = ransac_angle_pnp(rays, WP, opts);
% 修正中心（旋转后对应横屏）

% 横屏(1920×1080)的角点 -> 竖屏(1080×1920)像素系


% ---------- 4) RANSAC (像素误差判内点) ----------
I = imread(calib_data.L{kk});
opts.Ishow = I;

[bestR, bestC, inliers, iterCount] = ransac_pnp_pixel(rays, WP, imgPts, calib_data.ocam_model, opts);
fprintf('RANSAC (pixel) finished after %d iterations, inliers=%d/%d\n', iterCount, numel(inliers), size(WP,1));


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



% 已知：R_WC 是 camera->world 的旋转矩阵
%eul_zxy = rotm2eul_ZXY(R_WC);     % [z x y] in radians
%z_zxy   = eul_zxy(1);
%x_zxy   = eul_zxy(2);
%y_zxy   = eul_zxy(3);

%yaw_deg = rad2deg(z_zxy);
%pitch_deg = rad2deg(x_zxy);
%roll_deg = rad2deg(y_zxy);

eulZYX = rotm2eul(R_WC);       % [yaw pitch roll] (rad)  world->camera, ZYX
yaw_deg   = rad2deg(eulZYX(1));
pitch_deg = rad2deg(eulZYX(2));
roll_deg  = rad2deg(eulZYX(3));

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

    %Rc   = [1 0 0; 0 -1 0; 0 0 -1];   % RotX(pi)
    %R_WC = R_WC * Rc;                 % 相机侧右乘
    %R_CW = R_WC.';                    % 同步更新
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
% dbg
%dbg.inliers = inliers;
%dbg.rmse_px = rmse_px;
%dbg.rmse_ang_deg = ang_err;
%dbg.C_B = C_B;
%dbg.R_CW = R_CW;




%fprintf('RANSAC inliers = %d / %d\n', numel(inliers), N);



% 可视化（可选）
if opts.verbose
    I = imread(calib_data.L{kk});
    figure(3); clf; imshow(I,[]); title(sprintf('Image %d',kk)); hold on;
    plot(imgPts(:,1), imgPts(:,2), 'r+'); hold off;

    % === 3D 视图 ===
    figure(4); clf; hold on; grid on; axis equal vis3d; view(40,25);
    set(gca,'FontSize',12);

    % 棋盘尺寸
    board_w = (n_sq_x)*squareSize;
    board_h = (n_sq_y)*squareSize;
    board_span = max(board_w, board_h);

    % 画棋盘平面
    BX = reshape(WP(:,1), n_sq_y+1, n_sq_x+1);
    BY = reshape(WP(:,2), n_sq_y+1, n_sq_x+1);
    BZ = zeros(size(BX));
    surf(BX, BY, BZ, 'FaceColor',[.92 .92 .92], 'EdgeColor',[.4 .4 .4], 'LineWidth',1.2);

    % 棋盘坐标系
    local_draw_frame(eye(4), board_span*0.45, 'B');

    % 相机视锥
    %plotCamera('Location', C_B, 'Orientation', R_WC, ...
    %           'Size', 0.18*board_span, 'Color','r', 'Opacity',0.1);

    % 相机自身坐标系
    O = C_B(:);

    S = diag([1 1 -1]);       % 关于 z=0 平面的镜像矩阵

    % 原相机轴（与你当前代码一致）
    ex = R_WC(:,1); 
    ey = R_WC(:,2); 
    ez = R_WC(:,3);          % 若希望与 plotCamera 视锥一致，通常用 -Zc
    
    % 仅把要画的箭头进行镜像（不移动相机位置）
    ey2 = -(S*ex);
    ex2 = -(S*ey);
    ez = S*ez;

    %ex = R_WC(:,1);  % 相机X轴
    %ey = R_WC(:,2);  % 相机Y轴
    %ez = -R_WC(:,3);  % 相机Z轴
    s_cam = 0.25*board_span;

    quiver3(O(1),O(2),O(3), s_cam*ex2(1), s_cam*ex2(2), s_cam*ex2(3), ...
            'LineWidth',2.5,'Color','r','MaxHeadSize',0.7);
    text(O(1)+1.05*s_cam*ex2(1), O(2)+1.05*s_cam*ex2(2), O(3)+1.05*s_cam*ex2(3), 'C_X','Color','r','FontWeight','bold');

    quiver3(O(1),O(2),O(3), s_cam*ey2(1), s_cam*ey2(2), s_cam*ey2(3), ...
            'LineWidth',2.5,'Color','g','MaxHeadSize',0.7);
    text(O(1)+1.05*s_cam*ey2(1), O(2)+1.05*s_cam*ey2(2), O(3)+1.05*s_cam*ey2(3), 'C_Y','Color','g','FontWeight','bold');

    quiver3(O(1),O(2),O(3), s_cam*ez(1), s_cam*ez(2), s_cam*ez(3), ...
            'LineWidth',2.5,'Color','b','MaxHeadSize',0.7);
    text(O(1)+1.05*s_cam*ez(1), O(2)+1.05*s_cam*ez(2), O(3)+1.05*s_cam*ez(3), 'C_Z','Color','b','FontWeight','bold');

    title(sprintf('Image %d | inliers=%d/%d | rmse_px=%.3g | rmse_ang=%.3g°', ...
        kk, numel(inliers), N, rmse_px, ang_err));
    xlabel('X_B'); ylabel('Y_B'); zlabel('Z_B');
    hold off;
end

% dbg
dbg.inliers = inliers; dbg.rmse_px = rmse_px; dbg.rmse_ang_deg = ang_err;
dbg.C_B = C_B; dbg.R_CW = R_CW;

end

% ===================== 辅助函数们 =====================

function [R_CW, C_B, inliers, iterCount] = ransac_pnp_pixel(rays, WP, imgPts_uv, ocam, opts)
% 基于 PnP 的候选姿态 + 像素重投影误差判内点（鱼眼 world2cam）
% 输入：
%   rays       : N×3 单位光线（只用于构造 imgNorm；也可由 imgPts_uv 直接算）
%   WP         : N×3 世界点
%   imgPts_uv  : N×2 观测像素 [u,v] = [col,row]
%   ocam       : ocam_model（含 world2cam 参数）
%   opts.pixThreshPx  : 像素阈值（默认 2.0）
%   opts.maxTrials/opts.confidence/opts.angThreshDeg 同前（仅 maxTrials/confidence 用）
% 输出：
%   R_CW, C_B  : 最优候选姿态
%   inliers    : 内点索引
%   iterCount  : 实际迭代次数

    if ~isfield(opts,'pixThreshPx') || isempty(opts.pixThreshPx)
        pxThr = 2.0;
    else
        pxThr = opts.pixThreshPx;
    end
    s = 6;                               % 最小子集
    N = size(WP,1);
    bestScore = -inf; inliers = 1:N;
    iterCount = 0;

    % 归一化平面“虚拟像素”，用于 PnP 候选
    xn = rays(:,1)./max(1e-12, rays(:,3));
    yn = rays(:,2)./max(1e-12, rays(:,3));
    imgNorm = [xn, yn];
    camParams = cameraParameters('IntrinsicMatrix', eye(3)); % K=I

    maxTrials = opts.maxTrials;
    for t = 1:maxTrials
        iterCount = iterCount + 1;

        % 随机 4 点子集
        idx = randsample(N, s, false);
        try
            [R_CW_cand, C_cand] = pnp_subset(imgNorm(idx,:), WP(idx,:), camParams);
        catch
            continue
        end

        % --- 用 world2cam 计算像素重投影误差 ---
        Pc = (R_CW_cand*(WP.' - C_cand(:))).';           % N×3
        % 单位化更稳（OCam 常以方向向量为输入）
        Pc = Pc ./ max(1e-12, vecnorm(Pc,2,2));
        Pc(Pc(:,3) > 0, :) = -Pc(Pc(:,3) > 0, :);   % 关键：确保 z<0
        
                           % -> [u,v]
        % 2) 确保朝前：若 z<0 则翻转（很多 world2cam 只接受前半球）
        %flip = Pc(:,3) < 0;
        %Pc(flip,:) = -Pc(flip,:);

        pred_vu = zeros(N,2);
        for i=1:N
            vu = world2cam(Pc(i,:).', ocam);             % [v;u]
            pred_vu(i,:) = vu.';
        end
        pred_uv = [pred_vu(:,2), pred_vu(:,1)];          % 转为 [u,v]
        %pred_uv =pred_vu;
        % 可能越界/NaN：过滤无效点（不计入内点）
        valid = all(isfinite(pred_uv),2);
        duv = pred_uv(valid,:) - imgPts_uv(valid,:);     % [u,v] 一致
        err = sqrt(sum(duv.^2, 2));                      % 像素误差
        inl_local = find(valid);                          % 有效点索引
        inl_local = inl_local(err <= pxThr);              % 阈值判内点
        score = numel(inl_local);

        if score > bestScore
            bestScore = score;
            inliers   = inl_local;
            R_CW      = R_CW_cand;
            C_B       = C_cand(:).';

            % --- 自适应终止：达到全内点或置信度上限 ---
            if bestScore == N                    % 所有有效点都内点
                Ishow = opts.Ishow;

                            % 只展示有效预测的那部分
                            obs_uv  = imgPts_uv(valid,:);      % 观测 [u,v]
                            pred_uv_v = pred_uv(valid,:);      % 预测 [u,v]
                            err_v   = err;                     % 对应误差（已经只针对 valid 计算）
                        
                            % 适当抽样，防止太密集（可选）
                            idx_all = 1:size(obs_uv,1);
                            if numel(idx_all) > 200
                                idx_all = round(linspace(1, numel(idx_all), 200));
                            end
                        
                            figure(99); clf; imshow(Ishow,[]); hold on;
                        
                            % 真实角点：绿色圆点
                            plot(obs_uv(idx_all,1),  obs_uv(idx_all,2),  'go', 'MarkerSize',5, 'LineWidth',1.2);
                        
                            % 重投影点：洋红色十字
                            plot(pred_uv_v(idx_all,1), pred_uv_v(idx_all,2), 'm+', 'MarkerSize',6, 'LineWidth',1.2);
                        
                            % 观测与预测的连线：黄色细线
                            for kk_=idx_all
                                line([obs_uv(kk_,1),  pred_uv_v(kk_,1)], ...
                                     [obs_uv(kk_,2),  pred_uv_v(kk_,2)], 'Color','y', 'LineWidth',0.5);
                            end
                        
                            % 标注少量索引（可选）
                            lab_idx = idx_all(1: max(1, floor(numel(idx_all)/20)) : end);
                            for kk_=lab_idx
                                text(pred_uv_v(kk_,1)+4, pred_uv_v(kk_,2), sprintf('%d', kk_), ...
                                    'Color','w','FontSize',8,'FontWeight','bold');
                            end
                        
                            % 标题显示误差统计
                            title(sprintf('RANSAC t=1  |  green=obs  magenta=pred  |  mean=%.1f  med=%.1f  p95=%.1f  max=%.1f px', ...
                                mean(err_v), median(err_v), prctile(err_v,95), max(err_v)));
                        
                            hold off; drawnow;
                        
                            % 如需保存叠加图（可选）
                            if isfield(opts,'save_overlay') && ~isempty(opts.save_overlay)
                                frame = getframe(gca);
                                imwrite(frame.cdata, opts.save_overlay);  % 例如 opts.save_overlay='overlay_debug.png'
                            end
                break;
            end
   
        end

            

    end

    % 若极端情况下没有任何有效内点，兜底返回一次全量 PnP
    %if bestScore <= 0
    %    [R_CW, C_B] = pnp_subset(imgNorm, WP, camParams);
    %    C_B = C_B(:).';
    %    inliers = (1:N).';
    %end
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
    'MaxReprojectionError', 0.5, 'Confidence', 99.99, 'MaxNumTrials', 5000);
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

rr = reshape(r, 2, []).';       % N×2
rmse_px = sqrt(mean(sum(rr.^2,2)));
%rmse_px = sqrt(mean(reshape(r,2,[]).^2,'all'));
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
function r = reproj_residual_pix(x, WP, uv, ocam, rays)
    [R, C] = unpack_pose(x);
    Pc = (R*(WP.' - C)).';
    Pc = Pc ./ max(1e-12, vecnorm(Pc,2,2));

    % 半球对齐（若你在 refine 阶段也能提供对应 rays）
    if nargin >= 5 && ~isempty(rays)
        sgn = sign(sum(Pc .* rays, 2));
        sgn(sgn==0) = 1;
        Pc = Pc .* sgn;
    end

    N = size(Pc,1);
    pred_vu = zeros(N,2);
    for i=1:N
        vu = world2cam(Pc(i,:).', ocam);   % [v;u]
        pred_vu(i,:) = vu.';
    end
    pred_uv = [pred_vu(:,2), pred_vu(:,1)];  % -> [u,v]

    valid = all(isfinite(pred_uv),2);
    pred_uv = pred_uv(valid,:); uv = uv(valid,:);
    r = reshape((pred_uv - uv).', [], 1);
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

function eul = rotm2eul_ZXY(R)
% eul = [z x y] (radians), for sequence ZXY: R = Rz(z)*Rx(x)*Ry(y)
R = double(R);
clamp = @(t) max(-1, min(1, t));

% x from R(3,2) = sin(x)
x = asin( clamp(R(3,2)) );
cx = cos(x);

if abs(cx) > 1e-8
    % regular case
    % y from R(3,1) = -cx*sin(y), R(3,3)= cx*cos(y)
    y = atan2( -R(3,1), R(3,3) );
    % z from R(1,2) = -sin(z)*cx, R(2,2)= cos(z)*cx
    z = atan2( -R(1,2), R(2,2) );
else
    % gimbal lock: x ≈ ±pi/2, set y = 0 and recover z from R(2,1),R(1,1)
    y = 0;
    z = atan2( R(2,1), R(1,1) );
end

eul = [z, x, y];
end
