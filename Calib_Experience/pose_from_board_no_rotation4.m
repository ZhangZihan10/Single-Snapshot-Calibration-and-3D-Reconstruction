%%只在虚拟成功

function [C_B, R_CW, T_BC, dbg] = pose_from_board_no_rotation4( ...
    calib_data, kk, squareSize, use_corner_find, camAnglesDeg, yUp_user)
% 在已知相机绕自身轴 camAnglesDeg 的情况下，通过棋盘求相机位置
% 同时绘制棋盘和相机姿态

if nargin < 6 || isempty(yUp_user), yUp_user = false; end %#ok<NASGU>
if nargin < 5 || isempty(camAnglesDeg), camAnglesDeg = [0 0 0]; end
if nargin < 4 || isempty(use_corner_find), use_corner_find = true; end
if nargin < 3 || isempty(squareSize)
    if isfield(calib_data,'dX'), squareSize = calib_data.dX/1000; else, squareSize = 0.02; end
end

% 预置输出
C_B  = [NaN NaN NaN];
R_CW = eye(3);
T_BC = eye(4);
dbg  = struct('rmse',NaN,'N',0,'C_B',C_B,'R_CW',R_CW);

try
    % ---------- 1) 角点 ----------
    [cb, Xp, Yp] = get_checkerboard_cornersUrban9(kk, use_corner_find, calib_data);
    assert(cb==1, 'Corner detection failed.');
    Xp = Xp(:); Yp = Yp(:);
    imgPts = [Yp, Xp];

    % ---------- 2) 棋盘 3D ----------
    n_sq_x = calib_data.n_sq_x; n_sq_y = calib_data.n_sq_y;
    [Xg, Yg] = meshgrid(0:n_sq_x, 0:n_sq_y);
    WP = [Xg(:)*squareSize, Yg(:)*squareSize, zeros(numel(Xg),1)];
    N  = min(size(WP,1), size(imgPts,1));
    WP = WP(1:N,:); imgPts = imgPts(1:N,:);

    % ---------- 3) 相机自转矩阵 ----------
    a  = deg2rad(camAnglesDeg(:).');
    %a(3) = -a(3); 
    Rx = [1 0 0; 0 cos(a(1)) -sin(a(1)); 0 sin(a(1)) cos(a(1))];
    Ry = [cos(a(2)) 0 sin(a(2)); 0 1 0; -sin(a(2)) 0 cos(a(2))];
    Rz = [cos(a(3)) -sin(a(3)) 0; sin(a(3)) cos(a(3)) 0; 0 0 1];
    R_cam = Rz * Ry * Rx;

    % ---------- 4) 去旋转角点（旋转光线） ----------
    rays = cam2world_ocam_batch(imgPts, calib_data.ocam_model);
    if mean(rays(:,3)) < 0, rays = -rays; end
    rays_prime = (R_cam' * rays.').';

    % ---------- 5) PnP ----------
    xn = rays_prime(:,1)./rays_prime(:,3);
    yn = rays_prime(:,2)./rays_prime(:,3);
    Kvirt = [1000 0 0; 0 1000 0; 0 0 1];
    imgVirt = [xn*Kvirt(1,1), yn*Kvirt(2,2)];
    camParams = cameraParameters('IntrinsicMatrix',Kvirt');

    [R_CW_prime, camLoc] = estimateWorldCameraPose(imgVirt, WP, camParams, ...
        'MaxReprojectionError',3,'Confidence',99.9,'MaxNumTrials',3000);

    C_B  = camLoc(:).';
    R_CW = R_cam * R_CW_prime;
    %R_CW = R_CW_prime;   % 不再 R_cam * R_CW_prime


    % 1) 打印并保存旋转矩阵
R_WC = R_CW';   

disp('R_CW (world->camera) ='); disp(R_CW);
disp('R_WC (camera->world) ='); disp(R_WC);

% 2) 绕“世界 z 轴”的旋转角（yaw, ZYX欧拉）
% R_WC = Rz(yaw) * Ry(pitch) * Rx(roll)
yaw_z_world = atan2(R_WC(2,1), R_WC(1,1));        % rad
yaw_z_world_deg = rad2deg(yaw_z_world);
fprintf('Yaw about WORLD z-axis = %.3f deg\n', yaw_z_world_deg);

% 3) 相机 z 轴与世界 z 轴的夹角（倾角）
% 相机z轴在世界系中的方向是 R_WC(:,3)
cz_world = R_WC(:,3);
tilt_z_rad = acos( max(-1,min(1, dot(cz_world, [0;0;1]) )) );
tilt_z_deg = rad2deg(tilt_z_rad);
fprintf('Angle between camera-z and world-z (tilt) = %.3f deg\n', tilt_z_deg);




    T_BC = eye(4);
    T_BC(1:3,1:3) = R_CW;
    T_BC(1:3,4)   = C_B(:);

    % ---------- 6) 重投影误差 ----------
    rmse_pix = NaN;
    if exist('world2cam','file')==2
        Pc = (R_CW*(WP.' - C_B(:))).';
        uv = zeros(N,2);
        for i=1:N, uv(i,:) = world2cam(Pc(i,:).', calib_data.ocam_model).'; end
        e = uv - imgPts; rmse_pix = sqrt(mean(sum(e.^2,2)));
    end
    dbg.rmse = rmse_pix; dbg.N = N; dbg.C_B = C_B; dbg.R_CW = R_CW;

   % ---------- 7) 可视化 ----------
I = imread(calib_data.L{kk});
figure(2); clf; imshow(I,[]); title(sprintf('Image %d',kk)); hold on;
plot(imgPts(:,1), imgPts(:,2), 'r+'); hold off;

figure(3); clf; hold on; grid on; axis equal; view(40,25);
BX = reshape(WP(:,1), n_sq_y+1, n_sq_x+1);
BY = reshape(WP(:,2), n_sq_y+1, n_sq_x+1);
surf(BX, BY, zeros(size(BX)), 'FaceColor',[.9 .9 .9], 'EdgeColor',[.6 .6 .6]);
local_draw_frame(eye(4), squareSize*0.5, 'B');


R_WC = R_CW';



plotCamera('Location', C_B, 'Orientation', R_WC, ...
           'Size', squareSize*1.2, 'Color','r');

% === 新增：计算欧拉角（世界系 ZYX） ===
yaw   = atan2( R_WC(2,1), R_WC(1,1) );
pitch = asin( -R_WC(3,1) );
roll  = atan2( R_WC(3,2), R_WC(3,3) );

yaw_deg   = rad2deg(yaw);
pitch_deg = rad2deg(pitch);
roll_deg  = rad2deg(roll);

% 文字标注位置与内容
offset = 0.08*max(squareSize, 1);              % 依据棋盘格尺寸设置个相对偏移
p_txt  = C_B + offset*[1 1 1];                 % 文本放在相机旁边
label  = sprintf('roll_x = %.1f°\npitch_y = %.1f°\nyaw_z = %.1f°', ...
                 roll_deg, pitch_deg, yaw_deg);

% 画一条引线到相机位置（可选）
plot3([C_B(1) p_txt(1)], [C_B(2) p_txt(2)], [C_B(3) p_txt(3)], 'k-', 'LineWidth', 1);

% 文本框
text(p_txt(1), p_txt(2), p_txt(3), label, ...
    'FontSize', 10, 'Color', 'k', ...
    'BackgroundColor', [1 1 1], 'EdgeColor', [0.3 0.3 0.3], 'Margin', 4, ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');

% 画相机的局部坐标轴（长度与棋盘格相关，便于看方向）
axis_len = 0.5 * squareSize;
ex = R_WC(:,1);  ey = R_WC(:,2);  ez = R_WC(:,3);
quiver3(C_B(1), C_B(2), C_B(3), axis_len*ex(1), axis_len*ex(2), axis_len*ex(3), ...
        0, 'LineWidth', 2);  % 相机X（颜色由当前colormap控制）
quiver3(C_B(1), C_B(2), C_B(3), axis_len*ey(1), axis_len*ey(2), axis_len*ey(3), ...
        0, 'LineWidth', 2);
quiver3(C_B(1), C_B(2), C_B(3), axis_len*ez(1), axis_len*ez(2), axis_len*ez(3), ...
        0, 'LineWidth', 2);
text(C_B(1)+axis_len*ex(1), C_B(2)+axis_len*ex(2), C_B(3)+axis_len*ex(3), 'x_c', 'Color','k');
text(C_B(1)+axis_len*ey(1), C_B(2)+axis_len*ey(2), C_B(3)+axis_len*ey(3), 'y_c', 'Color','k');
text(C_B(1)+axis_len*ez(1), C_B(2)+axis_len*ez(2), C_B(3)+axis_len*ez(3), 'z_c', 'Color','k');

% 也把欧拉角存进 dbg（便于命令行拿数值）
dbg.eulerZYX_world_deg = [roll_deg, pitch_deg, yaw_deg];


catch ME
    dbg.error = ME.message;
end
end

% --- 工具 ---
function rays = cam2world_ocam_batch(uv, ocam)
N = size(uv,1); rays = zeros(N,3);
for k = 1:N, rays(k,:) = cam2world([uv(k,1); uv(k,2)], ocam).'; end
rays = rays ./ vecnorm(rays,2,2);
if mean(rays(:,3)) < 0, rays = -rays; end
end

function local_draw_frame(T, s, tag)
O=T(1:3,4); X=O+s*T(1:3,1); Y=O+s*T(1:3,2); Z=O+s*T(1:3,3);
plot3([O(1) X(1)],[O(2) X(2)],[O(3) X(3)],'r-','LineWidth',2); text(X(1),X(2),X(3),[tag,'_X'],'Color','r');
plot3([O(1) Y(1)],[O(2) Y(2)],[O(3) Y(3)],'g-','LineWidth',2); text(Y(1),Y(2),Y(3),[tag,'_Y'],'Color','g');
plot3([O(1) Z(1)],[O(2) Z(2)],[O(3) Z(3)],'b-','LineWidth',2); text(Z(1),Z(2),Z(3),[tag,'_Z'],'Color','b');
plot3(O(1),O(2),O(3),'ko','MarkerFaceColor','y');
end
