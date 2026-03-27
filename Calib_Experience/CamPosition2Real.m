
%% 真实 单张校准相机，激光面，定位
clc
clear all

global ocam_model;
global Cent;
global Cent1;
global cam_pitch_opt;
global cam_roll_opt;
global cam_yaw_opt;
global las_pitch_opt;
global las_roll_opt;
global Cube_width;

tic; % 开始计时

%load('Omni_Calib_Results1.mat');
load('Omni_Calib_Results_Real2old2.mat'); %旧的相机
%load('Omni_Calib_Results_Real2.mat'); %旧的相机
%load('Omni_Calib_Results_Real.mat'); %新的相机
%load('Omni_Calib_Results_Sim.mat'); % Calib parameters
ocam_model = calib_data.ocam_model; % Calib parameters

%图像增强
%img=imread('testr9.jpg'); % read image
%up_scale=3;
%im_h = SRCNN_single_image(img, up_scale);
% 保存增强结果
%enhanced_name = 'testr9E.jpg';
%imwrite(im_h, enhanced_name);

i = calib_data.n_ima;
calib_data.L(i+1)={'testr31_1.jpg'};
%calib_data.L{i+1} = enhanced_name;
use_corner_find=1;
% 已有 calib_data（包含 ocam_model, n_sq_x, n_sq_y），并把新图像路径放进 L{1}
I1_ = imread('testr31_1.jpg'); % read image

%squareSize = 0.116;
squareSize = 0.026;           % 单位米
%[C_B, R_WC, T_BC, info] = pose_from_board_no_rotation5(calib_data, i+1, squareSize, true);

opts = struct('angThreshDeg',0.5,'maxTrials',3000,'confidence',0.999,'refine','fisheye','verbose',true, 'pixThreshPx', 2.5);

[C_B, R_CW, T_BC, dbg, roll_deg, pitch_deg, yaw_deg] = pose_from_board_no_rotation7_3(calib_data, i+1, squareSize, true, opts);
%[C_B, R_WC, T_BC, dbg] = pose_from_board_no_rotation2(calib_data, i+1, squareSize, true);

%eul = rotm2eul(R_WC,'ZYX');            % [yaw pitch roll] in radians
%yaw_deg   = rad2deg(eul(1));     % 绕 Z（棋盘法线）的角
%pitch_deg = rad2deg(eul(2));     % 绕 Y 轴的角
%roll_deg  = rad2deg(eul(3));     % 绕 X 轴的角

disp('相机在棋盘(=世界)坐标系的位置 C_B ='); disp(C_B);
disp('姿态 R_CW ='); disp(R_CW);%world → camera 的旋转矩阵， 把世界坐标转到相机坐标


% for las
cam_pitch_opt=roll_deg;
cam_roll_opt=pitch_deg;
%cam_yaw_opt=-180-yaw_deg;
%cam_yaw_opt=180+yaw_deg;
  cam_yaw_opt=-yaw_deg;

%cam_roll_opt=roll_deg-180;%X
%cam_pitch_opt= pitch_deg;%Y
  %cam_yaw_opt=(180-yaw_deg);
%cam_yaw_opt=yaw_deg+180;%Z

Guess = 0;
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');


Cent = []; 
Cent1 = []; 
%I_ = las_segm(I1_,-80);
%[xx_,zz_,idx]=blue_blobs(I_,I1_);
% check whether number of blobs correct or not
idx=10;

if (idx ~= 8)
    fprintf('自动检测到 %d 个点，改用手动点选...\n', idx);
    % ——选择 A/B/C 其中之一——
    N = 8;
    figure; imshow(I1_); title('按顺序点击 8 个点'); hold on;
    [xc, yc] = ginput(N);
    pts = [xc(:) yc(:)];
    pts = sortrows(pts,1);
    for i = 1:2:N-1
        if pts(i,2) < pts(i+1,2)
            pts([i i+1],:) = pts([i+1 i],:);
        end
    end
    xx_ = pts(:,1);
    zz_ = pts(:,2);
    idx = N;
else
    % 用自动结果
    xx_ = xx_auto;
    zz_ = zz_auto;
    idx = idx_auto;
end
% fitting laser points for each side of the target
%img1 = las_segm(I1_,108); %imshow(img1)
%img1 = bwmorph(img1,'skel',Inf);
%img1=LaserFind2(I1_);

img1 = LaserFind4(I1_);
[Cent, Cent1]=intersect_blobs(img1,zz_,xx_); % Cent1,2-h, Cent2,3-v
% laser plane calibration
las_pitch_opt = fmincon(@objective_pitch_las,Guess,[],[],[],[],-30,30,...
    @constraint,options);
las_roll_opt = fmincon(@objective_roll_las,Guess,[],[],[],[],-30,30,...
    @constraint,options);

%las_pitch_opt = fminbnd(@objective_pitch_las2, -30, 30);
%las_roll_opt = fminbnd(@objective_roll_las2, -30, 30);



%%
%Cube_width = 320;
Cube_width = 305;
%Cube_width = 500;
las_dist_opt = fmincon(@objective_dist_las,Guess,[],[],[],[],[],[],...
    @constraint,options);

elapsed_time = toc; % 结束计时并获取运行时间
disp(['代码运行时间为：', num2str(elapsed_time), ' 秒']); % 显示运行时间

%% test fo paper   las_pitch_opt=-1;las_roll_opt=-3;las_dist_opt=228;
[x,y] = mapping_points(Cent,cam_roll_opt,cam_pitch_opt,cam_yaw_opt,...
    las_roll_opt,las_pitch_opt,las_dist_opt,ocam_model);
[x1,y1] = mapping_points(Cent1,cam_roll_opt,cam_pitch_opt,cam_yaw_opt,...
    las_roll_opt,las_pitch_opt,las_dist_opt,ocam_model);
figure
scatter(x,y,5,'b.'); % Laser intersections
hold on  
scatter(x1,y1,5,'b.'); % Laser intersections
scatter(0,0,'b*'); % Robot location
hold off

%
I2 = imread('testr31z2.jpg'); % read image
%I2 = imread('0001.jpg');
img2 = LaserFind4(I2);
%img2 = las_segm(I2,60);

[x2,y2] = mapping(img2,cam_roll_opt,cam_pitch_opt,cam_yaw_opt,...
    las_roll_opt,las_pitch_opt,las_dist_opt,ocam_model);
figure
scatter(x2,y2,5,'b.'); % Laser intersections