clc
clear all
load('Omni_Calib_Results_Sim.mat'); % Calib parameters
ocam_model = calib_data.ocam_model; % Calib parameters
i = calib_data.n_ima;
calib_data.L(i+1)={'TestImages/image.jpg'};
use_corner_find=1;
[callBack,Xp_abs_,Yp_abs_] = ...
    get_checkerboard_cornersUrban(i+1,use_corner_find,calib_data);
Xt = calib_data.Xt;
Yt = calib_data.Yt;
imagePoints = [Yp_abs_,Xp_abs_];
% first image extrinsic
[RRfin,ss]=calibrate(Xt, Yt, Xp_abs_, Yp_abs_, ocam_model);
RRfin_=FindTransformMatrix(Xp_abs_, Yp_abs_, Xt, Yt, ocam_model, RRfin);
% find distance
Y1 = RRfin_(1,3)
%% second image
i = calib_data.n_ima;
calib_data.L(i+1)={'TestImages/image1.jpg'};
use_corner_find=1;
[callBack,Xp_abs_,Yp_abs_] = ...
    get_checkerboard_cornersUrban(i+1,use_corner_find,calib_data);
Xt = calib_data.Xt;
Yt = calib_data.Yt;
imagePoints = [Yp_abs_,Xp_abs_];
% first image extrinsic
[RRfin,ss]=calibrate(Xt, Yt, Xp_abs_, Yp_abs_, ocam_model);
RRfin1_=FindTransformMatrix(Xp_abs_, Yp_abs_, Xt, Yt, ocam_model, RRfin);
% find distance
Y2 = RRfin1_(2,3)

%% obtain camera orientation and distance to patterns from camera
[M,M1] = show_patterns(0,0,0,i,RRfin_,RRfin1_,calib_data); % 0,0,0 – means that camera is not rotated (we don’t know its rotation)
% first pattern
angle_y = atand(diff([M(3,9),M(3,1)])/diff([M(1,9),M(1,1)]));
angle_z = atand(diff([M(2,9),M(2,1)])/diff([M(1,9),M(1,1)]));
angle_x = atand(diff([M(2,46),M(2,1)])/diff([M(3,46),M(3,1)]));
% second pattern
angle_y1 = atand(diff([M1(1,46),M1(1,1)])/diff([M1(3,46),M1(3,1)]));
angle_z1 = atand(diff([M1(1,9),M1(1,1)])/diff([M1(2,9),M1(2,1)]));
angle_x1 = atand(diff([M1(3,9),M1(3,1)])/diff([M1(2,9),M1(2,1)]));
% mean value with regards to the two patterns
camX = sign(angle_x)*(abs(angle_x)+abs(angle_x1))/2;
camY = sign(angle_y)*(abs(angle_y)+abs(angle_y1))/2;
camZ = sign(angle_z)*(abs(angle_z)+abs(angle_z1))/2;