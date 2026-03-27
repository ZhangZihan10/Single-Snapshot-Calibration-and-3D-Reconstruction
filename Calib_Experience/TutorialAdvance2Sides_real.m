% Parameters from Unity: 
% Camera: X=-3.5, Y=1, Z=2.5 | Laser: X=0.5, Y=2 | Laser Distance = 950
% Initialize
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

%load('Omni_Calib_Results_Sim.mat');
load('Omni_Calib_Results_Real2old2.mat'); %旧的相机
%load('Omni_Calib_Results_Real2.mat'); %旧的相机
ocam_model = calib_data.ocam_model;
%% Border extraction
I1_ = imread('testr31_2_1.jpg'); % read image  figure;imshow(I1_)
BW = BWBorder_Real(I1_); % extract border between Black & White regions
figure
imshow(~BW);



% extract border for each side of the box
[Cent,Cent1,idx]=border_blobs_R(BW);%
% check whether number of blobs correct or not
if (idx ~= 2)
    clear;
end
% camera calibration
Guess = 1;
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
cam_pitch_opt = fmincon(@objective_pitch_cam,Guess,[],[],[],[],-30,30,...
    @constraint,options);
%
cam_yaw_opt = fmincon(@objective_yaw_cam,Guess,[],[],[],[],-30,30,...
    @constraint,options);
%
cam_roll_opt = fmincon(@objective_roll_cam,Guess,[],[],[],[],-30,30,...
    @constraint,options);
% calib case
[x,y] = mapping_points(Cent,cam_roll_opt,cam_pitch_opt,cam_yaw_opt,...
    0,0,1,ocam_model);
[x1,y1] = mapping_points(Cent1,cam_roll_opt,cam_pitch_opt,cam_yaw_opt,...
    0,0,1,ocam_model);
scatter(x,y,5,'b.'); % border intersections
hold on
scatter(x1,y1,5,'b.'); % border intersections
scatter(0,0,'b*'); % Robot location

% for las  %cam_pitch_opt=-5;cam_roll_opt=3;cam_yaw_opt=4;
Cent = []; 
Cent1 = []; 
%I_ = las_segm(I1_,-80);
%[xx_,zz_,idx]=blue_blobs(I_,I1_);
% check whether number of blobs correct or not
idx=10;

I2_=imread('testr31_2.jpg');

if (idx ~= 8)
    fprintf('自动检测到 %d 个点，改用手动点选...\n', idx);
    % ——选择 A/B/C 其中之一——
    N = 8;
    figure; imshow(I2_); title('按顺序点击 8 个点'); hold on;
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
img1 = las_segm(I2_,70); %imshow(img1)
%img1 = bwmorph(img1,'skel',Inf);
%img1=LaserFind4(I1_);

%img1 = LaserFind4(I2_);
[Cent, Cent1]=intersect_blobs(img1,zz_,xx_); % Cent1,2-h, Cent2,3-v
% laser plane calibration
las_pitch_opt = fmincon(@objective_pitch_las,Guess,[],[],[],[],-30,30,...
    @constraint,options);
las_roll_opt = fmincon(@objective_roll_las,Guess,[],[],[],[],-30,30,...
    @constraint,options);
%%
%Cube_width = 1292.5;
Cube_width = 300;
las_dist_opt = fmincon(@objective_dist_las,Guess,[],[],[],[],[],[],...
    @constraint,options);%las_dist_opt=450;


elapsed_time = toc; % 结束计时并获取运行时间
disp(['代码运行时间为：', num2str(elapsed_time), ' 秒']); % 显示运行时间
%% test fo paper
%cam_pitch_opt=13; cam_roll_opt =10; cam_yaw_opt =1; 
% las_pitch_opt =1; las_roll_opt =-2; las_dist_opt=240;


[x,y] = mapping_points(Cent,cam_roll_opt,cam_pitch_opt,cam_yaw_opt,...
    las_roll_opt,las_pitch_opt,las_dist_opt,ocam_model);
[x1,y1] = mapping_points(Cent1,cam_roll_opt,cam_pitch_opt,cam_yaw_opt,...
    las_roll_opt,las_pitch_opt,las_dist_opt,ocam_model);
scatter(x,y,5,'b.'); % Laser intersections
hold on
scatter(x1,y1,5,'b.'); % Laser intersections
scatter(0,0,'b*'); % Robot location

I2 = imread('testr31z2.jpg'); % read image
img2 = LaserFind4(I2);
[x2,y2] = mapping(img2,cam_roll_opt,cam_pitch_opt,cam_yaw_opt,...
    las_roll_opt,las_pitch_opt,las_dist_opt,ocam_model);
figure
scatter(x2,y2,5,'b.'); % Laser intersections


%% 创建输出图像（拷贝原图）
I_overlay = imread('C:\论文\全向视觉畸变校准\图片\校准实验虚拟/image19_2n.jpg');
BW = imdilate(BW, strel('disk',1));
img1 = imdilate(img1, strel('disk',1));
% --- 显示 BW 结果为绿色 (0,255,0) ---
greenMask = BW > 0;
I_overlay(:,:,1) = I_overlay(:,:,1) .* uint8(~greenMask);            % R=0
I_overlay(:,:,2) = max(I_overlay(:,:,2),  uint8(255 * greenMask));   % G=255
I_overlay(:,:,3) = I_overlay(:,:,3) .* uint8(~greenMask);            % B=0
% --- 显示 img1 结果为蓝色 (0,0,255) ---
blueMask = img1 > 0;
I_overlay(:,:,1) = I_overlay(:,:,1) .* uint8(~blueMask);             % R=0
I_overlay(:,:,2) = I_overlay(:,:,2) .* uint8(~blueMask);             % G=0
I_overlay(:,:,3) = max(I_overlay(:,:,3), uint8(255 * blueMask));     % B=255
% 显示结果
figure; imshow(I_overlay);
title('BW = Green, Laser = Blue');
