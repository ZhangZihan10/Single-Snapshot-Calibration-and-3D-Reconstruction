%%

function [C_B, R_CW, T_BC, dbg] = pose_from_board_no_rotation3( ...
        calib_data, kk, squareSize, anglesDeg, use_corner_find)
% camX/camY/camZ 为“绕相机轴”的角度(度)。自动在常见内禀顺序中择优，求相机相对棋盘位姿。

if nargin < 5, use_corner_find = true; end
if nargin < 4 || isempty(anglesDeg), anglesDeg = [0 0 0]; end
if nargin < 3 || isempty(squareSize)
    if isfield(calib_data,'dX'), squareSize = calib_data.dX/1000; else, squareSize = 0.02; end
end

% ---------- 1) 角点 ----------
[cb, Xp, Yp] = get_checkerboard_cornersUrban9(kk, use_corner_find, calib_data);
if cb~=1, error('Corner detection failed.'); end
Xp = Xp(:);  Yp = Yp(:);
img = [Yp, Xp];  % [u,v]=[列,行]

% ---------- 2) 棋盘3D点 ----------
nx = calib_data.n_sq_x; ny = calib_data.n_sq_y;
[Xg, Yg] = meshgrid(0:nx, 0:ny);
Wp = [Xg(:)*squareSize, Yg(:)*squareSize, zeros(numel(Xg),1)];
N  = min(size(Wp,1), size(img,1));
Wp = Wp(1:N,:); img = img(1:N,:);

% ---------- 3) 像素 -> 归一化 ----------
rays = cam2world_ocam_batch(img, calib_data.ocam_model);
if mean(rays(:,3)) < 0, rays = -rays; end
xn = rays(:,1)./rays(:,3);  yn = rays(:,2)./rays(:,3);
m  = [xn, yn, ones(N,1)];

% ---------- 4) cam-axes 角 -> 候选 R_CW（内禀） -> 线性解 t_c -> 选 RMSE 最小 ----------
ax = deg2rad(anglesDeg(1)); ay = deg2rad(anglesDeg(2)); az = deg2rad(anglesDeg(3));
Rx = [1 0 0; 0 cos(ax) -sin(ax); 0 sin(ax) cos(ax)];
Ry = [cos(ay) 0 sin(ay); 0 1 0; -sin(ay) 0 cos(ay)];
Rz = [cos(az) -sin(az) 0; sin(az) cos(az) 0; 0 0 1];

cands = {Rz*Ry*Rx, Rx*Ry*Rz, Ry*Rx*Rz, Rz*Rx*Ry};  % 常见内禀顺序候选
best.rmse = inf;

for k = 1:numel(cands)
    R = cands{k}; r1 = R(:,1); r2 = R(:,2);
    A = zeros(3*N,3); b = zeros(3*N,1);
    for i=1:N
        Xi = Wp(i,1); Yi = Wp(i,2);
        p  = r1*Xi + r2*Yi;
        mx = m(i,1); my = m(i,2);
        S = [  0   -1   my;
               1    0  -mx;
              -my  mx    0  ];
        A(3*i-2:3*i,:) = S;
        b(3*i-2:3*i,1) = -S*p;
    end
    t = A\b;
    C = (-R.')*t;                       % 相机中心(棋盘系)
    Pc = (R*(Wp.' - C)).';              % 预测相机坐标
    uv_hat = [Pc(:,1)./Pc(:,3), Pc(:,2)./Pc(:,3)];
    rmse = sqrt(mean(sum((uv_hat - [xn yn]).^2,2)));   % 归一化平面 RMSE
    if rmse < best.rmse
        best.rmse = rmse; best.R = R; best.C = C(:).';
    end
end

R_CW = best.R; C_B = best.C; T_BC = eye(4); T_BC(1:3,1:3)=R_CW; T_BC(1:3,4)=C_B(:);
dbg.rmse_norm = best.rmse; dbg.N = N; dbg.C_B = C_B; dbg.R_CW = R_CW;

% ---------- 5) 可视化（保持你原风格） ----------
I = imread(calib_data.L{kk});
figure(2); clf; imshow(I,[]); title(sprintf('Image %d',kk)); hold on;
plot(img(:,1), img(:,2), 'r+');
if exist('draw_axes','file')==2, draw_axes(Xp, Yp, calib_data.n_sq_y); end
hold off;

yUp_user  = true;            % 你是按 y↑ 的直觉给角度 → 要做符号转换

[R_CW, T_BC] = apply_cam_axes_angles(R_CW, C_B, anglesDeg, yUp_user);

figure(3); clf; hold on; grid on; axis equal; view(40,25);
BX = reshape(Wp(:,1), ny+1, nx+1); BY = reshape(Wp(:,2), ny+1, nx+1);
surf(BX, BY, zeros(size(BX)), 'FaceColor',[.9 .9 .9], 'EdgeColor',[.6 .6 .6]);
local_draw_frame(eye(4), squareSize*0.5, 'B');
plotCamera('Location', C_B, 'Orientation', R_CW', 'Size', squareSize*1.2, 'Color','r');
xlabel('X_B'); ylabel('Y_B'); zlabel('Z_B');
title(sprintf('Image %d | cam-axes angles = [%.1f %.1f %.1f]^\\circ | rmse_n=%.3g', ...
      kk, anglesDeg(1), anglesDeg(2), anglesDeg(3), best.rmse));
hold off;
end

% ===== 你已有的两个小工具，贴在同文件底部即可 =====
function rays = cam2world_ocam_batch(uv, ocam)
N = size(uv,1); rays = zeros(N,3);
for k = 1:N, rays(k,:) = cam2world([uv(k,1); uv(k,2)], ocam).'; end
rays = rays ./ vecnorm(rays,2,2); if mean(rays(:,3))<0, rays = -rays; end
end
function local_draw_frame(T, s, tag)
O=T(1:3,4); X=O+s*T(1:3,1); Y=O+s*T(1:3,2); Z=O+s*T(1:3,3);
plot3([O(1) X(1)],[O(2) X(2)],[O(3) X(3)],'r-','LineW',2); text(X(1),X(2),X(3),[tag,'_X'],'Color','r');
plot3([O(1) Y(1)],[O(2) Y(2)],[O(3) Y(3)],'g-','LineW',2); text(Y(1),Y(2),Y(3),[tag,'_Y'],'Color','g');
plot3([O(1) Z(1)],[O(2) Z(2)],[O(3) Z(3)],'b-','LineW',2); text(Z(1),Z(2),Z(3),[tag,'_Z'],'Color','b');
plot3(O(1),O(2),O(3),'ko','MarkerFaceColor','y');
end

% ====== 在估计出 R_CW, C_B 之后，加入这段（如果有“绕相机轴”的角度） ======
function [R_CW_new, T_BC_new] = apply_cam_axes_angles(R_CW, C_B, anglesDeg, yUp_user)
    % anglesDeg = [camX, camY, camZ]（用户以“x右、y上、z前”的右手系给出）
    a = deg2rad(anglesDeg(:).');
    if yUp_user
        a(2) = -a(2);             % 用户y↑ -> 代码y↓ 的转换
    end
    Rx = [1 0 0; 0 cos(a(1)) -sin(a(1)); 0 sin(a(1)) cos(a(1))];
    Ry = [cos(a(2)) 0 sin(a(2)); 0 1 0; -sin(a(2)) 0 cos(a(2))];
    Rz = [cos(a(3)) -sin(a(3)) 0; sin(a(3)) cos(a(3)) 0; 0 0 1];

    % 以内禀(相机轴)顺序旋转：先绕X(roll)，再绕Y(pitch)，再绕Z(yaw)
    Rc = Rz * Ry * Rx;

    % 关键：绕相机轴 -> 左乘
    R_CW_new = Rc * R_CW;

    % 位置不变（绕相机光心自转）
    T_BC_new = eye(4);
    T_BC_new(1:3,1:3) = R_CW_new;
    T_BC_new(1:3,4)   = C_B(:);
end
