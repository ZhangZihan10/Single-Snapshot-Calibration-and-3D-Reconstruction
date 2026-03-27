% ===== 批量像素 -> 单位光线（基于 OCamCalib 的 cam2world）=====
function rays = cam2world_ocam_batch(uv, ocam)
% uv: N×2，列为 [u v] = [像素x(列) 像素y(行)]
% 返回: N×3 单位方向向量（相机坐标系）
    N = size(uv,1);
    rays = zeros(N,3);

    % 如果工具箱的 cam2world 不在路径里，给出清晰报错
    if exist('cam2world','file') ~= 2
        error(['未找到 OCamCalib 的 cam2world.m。请确保已 addpath 到 OCamCalib 工具箱，',...
               '或把该文件放到当前路径下。']);
    end

    for k = 1:N
        % OCamCalib 的 cam2world 接口：cam2world([u;v], ocam_model)
        r = cam2world([uv(k,1); uv(k,2)], ocam);
        rays(k,:) = r(:).';
    end

    % 归一化，稳妥起见
    nrm = sqrt(sum(rays.^2,2));
    nrm(nrm==0) = 1;
    rays = rays ./ nrm;
end
