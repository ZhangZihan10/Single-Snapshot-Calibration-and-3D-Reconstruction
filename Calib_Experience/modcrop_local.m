% -------------------- 辅助函数 --------------------
function I = modcrop_local(I, scale)
% 将图像裁剪为能被 scale 整除的尺寸
    sz = size(I);
    sz_mod = mod(sz(1:2), scale);
    I = I(1:end-sz_mod(1), 1:end-sz_mod(2), :);
end
