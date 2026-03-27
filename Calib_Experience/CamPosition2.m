clc
clear all
%load('Omni_Calib_Results_Sim.mat'); % Calib parameters
load('Omni_Calib_Results_Real2.mat');
ocam_model = calib_data.ocam_model; % Calib parameters
i = calib_data.n_ima;
%calib_data.L(i+1)={'test4.jpg'};
calib_data.L(i+1)={'testr8.jpg'};
use_corner_find=1;
% 已有 calib_data（包含 ocam_model, n_sq_x, n_sq_y），并把新图像路径放进 L{1}

squareSize = 0.116;           % 单位米
anglesDeg = [0, 0, 0];  % 你的已知角，单位：度。若只有X/Y，Z填0
[C_B, R_CW, T_BC, dbg] = pose_from_board_no_rotation4(calib_data, i+1, squareSize, true, anglesDeg);
disp(C_B)    % 就是相机在棋盘(世界)系的位置
disp('相机在棋盘(=世界)坐标系的位置 C_B ='); disp(C_B);
disp('姿态 R_WC ='); %disp(R_WC);
