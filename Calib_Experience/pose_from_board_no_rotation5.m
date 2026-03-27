%%

function [C_B, R_CW, T_BC, dbg] = pose_from_board_no_rotation5(calib_data, kk, squareSize, use_corner_find)
% 以“棋盘坐标系B=世界系”求相机位姿（板平放，无需额外地面姿态）
% 输出：
%   C_B   : 1x3，相机在棋盘系B中的位置
%   R_CW  : 3x3，world->camera 的旋转（棋盘->相机）
%   T_BC  : 4x4，^B T_C 齐次矩阵（棋盘到相机）
%   dbg   : 结构体，含 rmse / 使用的角点等

if nargin < 4, use_corner_find = true; end
if nargin < 3 || isempty(squareSize)
    % 你数据里 dX,dY=每格尺寸[mm]，这里转[m]
    if isfield(calib_data,'dX'), squareSize = calib_data.dX/1000; else, squareSize = 0.02; end
end

% ---------- 1) 角点检测（或直接取已有） ----------

        [cb, Xp, Yp] = get_checkerboard_cornersUrban9(kk, use_corner_find, calib_data);
    
    if cb~=1, error('Corner detection failed.'); end

Xp = Xp(:);  Yp = Yp(:);                  % 列向量
imagePoints = [Yp, Xp];                   % [u,v]=[列,行]

% ---------- 2) 按“图像编号顺序”构造棋盘3D点 ----------
% 你的左图是：左上为原点，先沿 X(水平) 递增，再沿 Y(竖直) 递增
n_sq_x = calib_data.n_sq_x;   n_sq_y = calib_data.n_sq_y;
[Xg, Yg] = meshgrid(0:n_sq_x, 0:n_sq_y);     % X=列, Y=行（与图像一致）
worldPoints_B = [Xg(:)*squareSize, Yg(:)*squareSize, zeros(numel(Xg),1)];

% 对齐数量
N = min(size(worldPoints_B,1), size(imagePoints,1));
worldPoints_B = worldPoints_B(1:N,:);
imagePoints   = imagePoints(1:N,:);

% ---------- 3) OCam 像素 -> 单位光线 -> 虚拟针孔像素 ----------
rays = cam2world_ocam_batch(imagePoints, calib_data.ocam_model); % N×3
% 归一化坐标
xn = rays(:,1)./rays(:,3);   yn = rays(:,2)./rays(:,3);
% 虚拟针孔内参（任取，只要一致）
Kvirt = [1000 0 0; 0 1000 0; 0 0 1];
imgVirt = [xn*Kvirt(1,1), yn*Kvirt(2,2)];
camParams = cameraParameters('IntrinsicMatrix',Kvirt');

% ---------- 4) PnP：相机相对棋盘的姿态 ----------
% MATLAB 文档：estimateWorldCameraPose 返回 R_CW(世界->相机), camLocation(世界坐标)
[R_CW, camLoc] = estimateWorldCameraPose(imgVirt, worldPoints_B, camParams, ...
                     'MaxReprojectionError', 3, 'Confidence', 99.9, 'MaxNumTrials', 3000);
C_B = camLoc(:)';                    % 相机在棋盘系的位置
T_BC = eye(4);  T_BC(1:3,1:3)=R_CW;  T_BC(1:3,4)=C_B(:);

% ---------- 5) 重投影检查（若有 world2cam 则用 OCam 前向投影） ----------
rmse_pix = NaN;
if exist('world2cam','file')==2
    Pc = (R_CW*(worldPoints_B.' - C_B(:))).';     % B->C:  Xc = R*(Xw - C)
    uv = zeros(N,2);
    for i=1:N
        uv_i = world2cam(Pc(i,:).', calib_data.ocam_model);
        uv(i,:) = uv_i(:).';
    end
    err = uv - imagePoints;
    rmse_pix = sqrt(mean(sum(err.^2,2)));
end
dbg.rmse = rmse_pix; dbg.N = N; dbg.C_B = C_B; dbg.R_CW = R_CW;


% ===== 计算相机最终旋转角（在可视化之前粘贴） =====
R_WC = R_CW.';   % camera -> world

% 世界系ZYX欧拉角：R_WC = Rz(yaw)*Ry(pitch)*Rx(roll)
yaw   = atan2( R_WC(2,1), R_WC(1,1) );
pitch = asin( -R_WC(3,1) );
roll  = atan2( R_WC(3,2), R_WC(3,3) );

yaw_deg   = -rad2deg(yaw);
pitch_deg = -rad2deg(pitch);
roll_deg  = rad2deg(roll);

% 相机光轴(+Zc)在棋盘平面上的投影与方位
zc_world = R_WC(:,3);                                  % 相机Z轴在世界系
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



% ---------- 6) 可视化 ----------
% 6.1 原图 + 角点 + 坐标轴
I = imread(calib_data.L{kk});
figure(2); clf; imshow(I,[]); title(sprintf('Image %d',kk)); hold on;
plot(imagePoints(:,1), imagePoints(:,2), 'r+');
if exist('draw_axes','file')==2
    draw_axes(Xp, Yp, calib_data.n_sq_y);   % 你原项目的绿色 O/X/Y
end
hold off;

% 6.2 棋盘系中的相机
figure(3); clf; hold on; grid on; axis equal; view(40,25);
% 画棋盘网格
BX = reshape(worldPoints_B(:,1), n_sq_y+1, n_sq_x+1);
BY = reshape(worldPoints_B(:,2), n_sq_y+1, n_sq_x+1);
surf(BX, BY, zeros(size(BX)), 'FaceColor',[.9 .9 .9], 'EdgeColor',[.6 .6 .6]);
% 坐标轴
if exist('draw_frame','file')~=2, draw_frame = @(T,s,tag) []; end %#ok<NASGU>
local_draw_frame(eye(4), squareSize*0.5, 'B');
% 相机（注意 plotCamera 期望的是 camera->world 旋转，所以传 R_CW'）
plotCamera('Location', C_B, 'Orientation', R_CW', 'Size', squareSize*1.2, 'Color','r');
xlabel('X_B'); ylabel('Y_B'); zlabel('Z_B');
ttl = sprintf('Image %d | rmse=%.3g px', kk, rmse_pix);
title(ttl); hold off;

end


function rays = cam2world_ocam_batch(uv, ocam)
% uv: N×2 [u v] = [列 行]
N = size(uv,1);
rays = zeros(N,3);
for k = 1:N
    r = cam2world([uv(k,1); uv(k,2)], ocam); % OCamCalib 的函数
    rays(k,:) = r(:).';
end
% 归一化（保险）
rays = rays ./ vecnorm(rays,2,2);
% 统一朝向：让 Z>0 指向前方（若需要）
if mean(rays(:,3)) < 0, rays = -rays; end
end


function local_draw_frame(T, s, tag)
O = T(1:3,4);
X = O + s*T(1:3,1); Y = O + s*T(1:3,2); Z = O + s*T(1:3,3);
plot3([O(1) X(1)],[O(2) X(2)],[O(3) X(3)],'r-','LineWidth',2); text(X(1),X(2),X(3),[tag,'_X'],'Color','r');
plot3([O(1) Y(1)],[O(2) Y(2)],[O(3) Y(3)],'g-','LineWidth',2); text(Y(1),Y(2),Y(3),[tag,'_Y'],'Color','g');
plot3([O(1) Z(1)],[O(2) Z(2)],[O(3) Z(3)],'b-','LineWidth',2); text(Z(1),Z(2),Z(3),[tag,'_Z'],'Color','b');
plot3(O(1),O(2),O(3),'ko','MarkerFaceColor','y');
end

