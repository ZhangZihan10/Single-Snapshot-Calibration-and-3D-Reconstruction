function camera_position(calib_data)
% CAMERA_POSITION  通过棋盘图像和标定结果估计相机位姿
%
% 输入:
%   calib_data : 标定数据结构（包含 ocam_model, n_sq_x, n_sq_y, dX 等字段）
%                图像名由用户在命令行输入
%
% 输出:
%   C_B   : 1x3，相机在棋盘(=世界)坐标系中的位置
%   R_WC  : 3x3，相机->世界的旋转矩阵
%   T_BC  : 4x4，棋盘到相机的齐次变换
%   dbg   : 调试信息结构体

% === 1) 自动识别棋盘格大小 ===
if isfield(calib_data,'dX') && ~isempty(calib_data.dX)
    squareSize = calib_data.dX / 1000;   % 毫米转米
else
    error('calib_data.dX 未定义，无法自动获取棋盘格大小');
end

% === 2) 让用户输入图像文件名 ===
imgName = input('Enter image name：','s');

% 在 calib_data.L 中查找图像
kk = [];
if isfield(calib_data,'L') && ~isempty(calib_data.L)
    for i = 1:numel(calib_data.L)
        [~,name,ext] = fileparts(calib_data.L{i});
        if strcmpi([name ext], imgName)
            kk = i;
            break;
        end
    end
end

% 如果没找到，就把图像名加入 calib_data.L
if isempty(kk)
    if ~isfield(calib_data,'L') || isempty(calib_data.L)
        calib_data.L = {};
    end
    kk = numel(calib_data.L) + 1;
    calib_data.L{kk} = imgName;
end

% === 3) 角点检测 + PnP求解 ===
use_corner_find = true;
opts = struct('angThreshDeg',1.0,'maxTrials',3000, ...
              'confidence',0.999,'refine','fisheye','verbose',true);

[C_B, R_CW, T_BC, dbg] = pose_from_board_no_rotation6(calib_data, kk, squareSize, use_corner_find, opts);

% === 4) 转换成 camera->world 旋转矩阵 ===
%R_WC = R_CW';

% === 5) 打印结果 ===
disp('相机在棋盘(=世界)坐标系的位置 C_B ='); disp(C_B);
disp('姿态 R_WC (camera->world) ='); disp(R_WC);

end
