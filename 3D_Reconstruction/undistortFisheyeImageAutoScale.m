function [J, camIntrinsics] = undistortFisheyeImageAutoScale(I, intrinsics, varargin)
% 自动尝试最大ScaleFactor去畸变，避免报错
% 输入参数同 undistortFisheyeImage

% 解析输入参数
p = inputParser;
addParameter(p,'OutputView','same', @(x) any(validatestring(x,{'same','full','valid'})));
addParameter(p,'ScaleFactor',1,@(x) isnumeric(x) && all(x>0));
addParameter(p,'FillValues',0);
addParameter(p,'Interp','bilinear', @(x) any(validatestring(x,{'bilinear','nearest','cubic'})));
parse(p,varargin{:});
outputView = p.Results.OutputView;
scaleFactor = p.Results.ScaleFactor;
fillValues = p.Results.FillValues;
interp = p.Results.Interp;

minScaleFactor = 0.5;
scaleStep = 0.05;

lastErr = [];

while scaleFactor >= minScaleFactor
    try
        J = undistortFisheyeImage(I, intrinsics, ...
            'OutputView', outputView, ...
            'ScaleFactor', scaleFactor, ...
            'FillValues', fillValues, ...
            'Interp', interp);
        camIntrinsics = []; % 官方函数不返回内参，这里可以空着
        return
    catch ME
        lastErr = ME;
        scaleFactor = scaleFactor - scaleStep;
    end
end

% 如果都失败了，抛出最后一个错误
rethrow(lastErr);

end
