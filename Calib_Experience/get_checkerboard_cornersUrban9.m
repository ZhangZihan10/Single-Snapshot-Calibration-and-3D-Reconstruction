function [callBack, Xp_abs, Yp_abs] = get_checkerboard_cornersUrban9(kk, use_corner_find, calib_data)
% 自动切换 + 融合版角点检测：
% 1) 原图->EXE（原方法），若可得全点则保留为 raw 结果；
% 2) 增强流程（放大+极性兜底+自适应窗口+单应补齐），作为 enh 结果（完整）；
% 3) 将 raw 与 enh 对齐到同一索引（基于 enh 完整网格），逐点比较：差大取 enh，差小取 raw，raw 缺失取 enh。
% 输出按原工程约定：
%   Yp_abs = 图像 x（列坐标，1-based）
%   Xp_abs = 图像 y（行坐标，1-based）

% ==== 默认输出 ====
callBack = 0;  Xp_abs = []; Yp_abs = [];

% ==== 清理残留 ====
fclose('all');
safe_del('autoCornerFinder/cToMatlab/cornerInfo.txt');
safe_del('autoCornerFinder/cToMatlab/cornersX.txt');
safe_del('autoCornerFinder/cToMatlab/cornersY.txt');
safe_del('autoCornerFinder/cToMatlab/error.txt');

% 期望角点
nx = calib_data.n_sq_x + 1;
ny = calib_data.n_sq_y + 1;
N_expected = nx * ny;

fprintf(1,'\nProcessing image %s...', calib_data.L{kk});
Iraw = imread(calib_data.L{kk});
if size(Iraw,3)==3, Igray = rgb2gray(Iraw); else, Igray = Iraw; end

%% (A) 原方法：原图->EXE，不缩放不取反
raw_ok = false; x_raw = []; y_raw = [];
[cb0, cx0, cy0] = run_exe_with_image(Igray, nx, ny, N_expected);
if cb0 == 1
    % 0->1 基
    cx0 = cx0 + 1;  cy0 = cy0 + 1;
    % 固定窗口亚像素（贴近原法）
    if use_corner_find
        wintx = max(3, round(size(Igray,1)/100)); winty = wintx;
        Is = im2single(Igray);
        for i = 1:size(cx0,1)
            for j = 1:size(cx0,2)
                if cx0(i,j) > 0 && cy0(i,j) > 0
                    xxi = cornerfinder([cx0(i,j); cy0(i,j)], Is, winty, wintx);
                    cx0(i,j) = xxi(1); cy0(i,j) = xxi(2);
                end
            end
        end
    end
    % 原法排序
    [x_ord0, y_ord0] = original_order_from_matrix(cx0-1, cy0-1, calib_data.n_sq_x, calib_data.n_sq_y);
    x_ord0 = x_ord0 + 1; y_ord0 = y_ord0 + 1;

    if numel(x_ord0) == N_expected
        raw_ok = true;
        x_raw = x_ord0(:);  % 列(x)
        y_raw = y_ord0(:);  % 行(y)
    else
        % 保留部分 raw 点，后续对齐时会按近邻匹配
        x_raw = x_ord0(:);
        y_raw = y_ord0(:);
    end
end

%% (B) 增强流程：放大 + 取反兜底 + 自适应窗口 + 单应补齐
scaleUp = 1.8;
Iup   = imgaussfilt(imresize(Igray, scaleUp, 'bicubic'), 0.8);
Iflip = imgaussfilt(imresize(imcomplement(Igray), scaleUp, 'bicubic'), 0.8);

[cb1, cx1, cy1] = run_exe_with_image(Iup,   nx, ny, N_expected);
[cb2, cx2, cy2] = run_exe_with_image(Iflip, nx, ny, N_expected);
if cb1==1, cx1 = cx1/scaleUp + 1; cy1 = cy1/scaleUp + 1; end
if cb2==1, cx2 = cx2/scaleUp + 1; cy2 = cy2/scaleUp + 1; end

[cx, cy, cb] = pick_better(cx1,cy1,cb1, cx2,cy2,cb2);
if cb ~= 1
    % 增强也失败：若 raw 完整就用 raw，否则失败
    if raw_ok
        Yp_abs = x_raw; Xp_abs = y_raw; callBack=1;
        plot_like_legacy(Igray, Xp_abs, Yp_abs, kk, calib_data.n_sq_y);
        fprintf(1,' Done (fallback raw)\n'); 
        return;
    else
        
        fprintf(1,' Image omitted -- Not all corners found\n'); return;
    end
end

% 自适应窗口（增强法）
P = [cx(cx>0), cy(cy>0)];
d_est = estimate_grid_pitch(P);
wintx = max(3, round(d_est/6));  winty = wintx;
if use_corner_find
    Is = im2single(Igray);
    for i=1:size(cx,1)
        for j=1:size(cx,2)
            if cx(i,j) > 0 && cy(i,j) > 0
                xxi = cornerfinder([cx(i,j); cy(i,j)], Is, winty, wintx);
                cx(i,j) = xxi(1); cy(i,j) = xxi(2);
            end
        end
    end
end

% 排序（增强法）
[x_enh, y_enh] = original_order_from_matrix(cx-1, cy-1, calib_data.n_sq_x, calib_data.n_sq_y);
x_enh = x_enh + 1; y_enh = y_enh + 1;

% 若增强检测点不足，单应性补齐
if numel(x_enh) < N_expected
    [x_enh, y_enh] = complete_with_homography(x_enh, y_enh, nx, ny);
end
if numel(x_enh) ~= N_expected
    fprintf(1,'\n[Error] Enhanced result incomplete: %d/%d\n', numel(x_enh), N_expected);
    if raw_ok
        Yp_abs = x_raw; Xp_abs = y_raw; callBack=1;
        plot_like_legacy(Igray, Xp_abs, Yp_abs, kk, calib_data.n_sq_y);
        fprintf(1,' Done (fallback raw)\n'); return;
    else
        return;
    end
end

% ======= 融合：将 raw 对齐到 enh 的索引，再逐点选择 =======
% 对齐（raw 可能缺点/顺序略乱）：用最近邻匹配 + 门限
thr_align = max(0.4*d_est, 2);  % 对齐半径
[x_raw_aln, y_raw_aln, raw_has] = align_raw_to_enhanced(x_raw, y_raw, x_enh, y_enh, thr_align);

% 逐点融合：差值阈值
thr_diff = min(max(0.10*d_est, 1.0), 5.0);  % 自适应且有上下限
x_final = x_enh; y_final = y_enh;           % 先用增强结果填满
for k = 1:N_expected
    if raw_has(k)
        dx = x_raw_aln(k) - x_enh(k);
        dy = y_raw_aln(k) - y_enh(k);
        if hypot(dx,dy) <= thr_diff
            % 差值小：用原方法（偏稳）
            x_final(k) = x_raw_aln(k);
            y_final(k) = y_raw_aln(k);
        else
            % 差值大：保留增强法（更稳健）
            % x_final,y_final 不变
        end
    else
        % 原方法没检到：用增强法（已是 enh）
    end
end

% 输出
Yp_abs = x_final(:);  % 列
Xp_abs = y_final(:);  % 行
callBack = 1;

% ====== 可视化（与原程序一致）======
plot_like_legacy(Igray, Xp_abs, Yp_abs, kk, calib_data.n_sq_y);

% ====== 持久化两套点（可注释）======
try
    assignin('base','raw_Xp_abs',set_cell(assignin_get('raw_Xp_abs'),kk,y_raw));
    assignin('base','raw_Yp_abs',set_cell(assignin_get('raw_Yp_abs'),kk,x_raw));
    assignin('base','enh_Xp_abs',set_cell(assignin_get('enh_Xp_abs'),kk,y_enh));
    assignin('base','enh_Yp_abs',set_cell(assignin_get('enh_Yp_abs'),kk,x_enh));
    assignin('base','final_Xp_abs',set_cell(assignin_get('final_Xp_abs'),kk,y_final));
    assignin('base','final_Yp_abs',set_cell(assignin_get('final_Yp_abs'),kk,x_final));
    outdir = fullfile('autoCornerFinder','corners_cache'); if ~exist(outdir,'dir'), mkdir(outdir); end
    save(fullfile(outdir,sprintf('corners_%03d.mat',kk)), ...
        'x_raw','y_raw','x_enh','y_enh','x_final','y_final','thr_align','thr_diff');
catch
end

fprintf(1,' Done (fused)\n');

end

% ================= 工具函数 =================

function safe_del(fname)
    if exist(fname,'file')
        try, delete(fname); catch, warning('无法删除文件: %s', fname); end
    end
end

function [callBack, cornersX, cornersY] = run_exe_with_image(I_for_exe, nx, ny, numCorners)
callBack = 0; cornersX=[]; cornersY=[];
tmpPng = fullfile('autoCornerFinder','tmp_preI.png');
imwrite(I_for_exe, tmpPng);
fid = fopen('./autoCornerFinder/pictures.txt','w'); fprintf(fid,'tmp_preI.png'); fclose(fid);
cd autoCornerFinder; cmd = sprintf('FindCorners.exe -w %d -h %d -m %d pictures.txt', nx, ny, numCorners);
if ~ispc, cmd = ['./' cmd]; end
callBack = system(cmd); cd ..
if callBack ~= 1, return; end
fi = fopen('autoCornerFinder/cToMatlab/cornerInfo.txt','r'); if fi==-1, callBack=0; return; end
ci = fscanf(fi,'%g %g',[1 2]); fclose(fi);
fx = fopen('autoCornerFinder/cToMatlab/cornersX.txt','r'); if fx==-1, callBack=0; return; end
cornersX = fscanf(fx,'%g %g',[ci(2) ci(1)]).'; fclose(fx);
fy = fopen('autoCornerFinder/cToMatlab/cornersY.txt','r'); if fy==-1, callBack=0; return; end
cornersY = fscanf(fy,'%g %g',[ci(2) ci(1)]).'; fclose(fy);
end

function [X, Y, cb] = pick_better(X1,Y1,cb1, X2,Y2,cb2)
score = @(X,Y) sum(X(:)>0 & Y(:)>0);
s1 = (cb1==1) * score(X1,Y1);
s2 = (cb2==1) * score(X2,Y2);
if s2 > s1, X=X2; Y=Y2; cb=cb2; else, X=X1; Y=Y1; cb=cb1; end
end

function d = estimate_grid_pitch(P)
if size(P,1)<4, d=8; return; end
D = pdist2(P,P); D = D + diag(inf(size(D,1),1));
Ds = sort(D,2,'ascend'); cand = Ds(:,1:6);
cand = cand(:); cand = cand(cand>0 & cand < prctile(cand,70));
if isempty(cand), d=8; else, d=median(cand); end
end

function [x_out, y_out] = original_order_from_matrix(cornersX0, cornersY0, n_sq_x, n_sq_y)
cornersX = cornersX0; cornersY = cornersY0;
numCorners = (n_sq_x+1)*(n_sq_y+1);
[rows, cols] = size(cornersX); deltaCols = 0;
startingCorner = [];
for i = 1:rows
    for j = 1:cols-1
        if cornersX(i,j) >= 0 && cornersY(i,j) >= 0 && cornersX(i,j+1) >= 0 && cornersY(i,j+1) >= 0
            startingCorner = [i,j]; break;
        end
    end
    if ~isempty(startingCorner), break; end
end
if isempty(startingCorner)
    [ii,jj] = find((cornersX>=0)&(cornersY>=0),1,'first');
    if isempty(ii), x_out=[]; y_out=[]; return; end
    startingCorner = [ii,jj];
end
min_i = startingCorner(1); min_j = startingCorner(2);
x = []; y = []; iteration = 0;
while true
    i = min_i + floor(iteration/(cols - deltaCols));
    j = mod((min_j - 1 + iteration), (cols - deltaCols)) + 1;
    iteration = iteration + 1;
    if i>rows, break; end
    x = [x, cornersX(i,j)]; y = [y, cornersY(i,j)];
    if iteration >= numCorners, break; end
end
n_cor_min = min(n_sq_x+1, n_sq_y+1); n_cor_max = max(n_sq_x+1, n_sq_y+1);
dxy = sqrt(diff(x).^2 + diff(y).^2); if isempty(dxy), x_out=[]; y_out=[]; return; end
hmin = zeros(1, n_cor_max*n_cor_min - 1); hmin(1:n_cor_min:end) = 1;
hmax = zeros(1, n_cor_max*n_cor_min - 1); hmax(1:n_cor_max:end) = 1;
pmin = conv(hmin, dxy); pmax = conv(hmax, dxy);
if pmin(1) > pmax(1), inc_dir = n_cor_min; else, inc_dir = n_cor_max; end
xTemp = x; yTemp = y; area = zeros(1, max(1,numel(x)-1));
for k = 1:numel(area)
    xb = [xTemp(1,1:inc_dir-1), xTemp(1,inc_dir:inc_dir:end-inc_dir), xTemp(1,end:-1:end-inc_dir+2), xTemp(1,end-inc_dir+1:-inc_dir:1)];
    yb = [yTemp(1,1:inc_dir-1), yTemp(1,inc_dir:inc_dir:end-inc_dir), yTemp(1,end:-1:end-inc_dir+2), yTemp(1,end-inc_dir+1:-inc_dir:1)];
    area(k) = abs(trapz(xb, yb));
    xTemp = [xTemp(1,2:end), xTemp(1,1)]; yTemp = [yTemp(1,2:end), yTemp(1,1)];
end
[~,shift] = max(area); shift = shift - 1;
if shift > 0
    x = [x(1,1+shift:end), x(1,1:shift)];
    y = [y(1,1+shift:end), y(1,1:shift)];
end
if n_sq_x ~= n_sq_y
    n_cor_x = n_sq_x + 1; n_cor_y = n_sq_y + 1;
    if numel(x) >= max(n_cor_x, n_cor_y)+1
        dist1 = (x(1,n_cor_x)-x(1,n_cor_x+1))^2 + (y(1,n_cor_x)-y(1,n_cor_x+1))^2;
        dist2 = (x(1,n_cor_y)-x(1,n_cor_y+1))^2 + (y(1,n_cor_y)-y(1,n_cor_y+1))^2;
        if dist1 > dist2
            xTemp = x; yTemp = y; L = numel(x); iterMult = n_cor_x; iterOffset = 0;
            for i = 1:L
                j = mod(i-1, n_cor_y) + 1;
                idx = j*iterMult - iterOffset; if idx> L, idx = L; end
                x(i) = xTemp(idx); y(i) = yTemp(idx);
                if j*iterMult > n_cor_x*(n_cor_y-1), iterOffset = iterOffset + 1; end
            end
        end
    end
end
x_out = x(:); y_out = y(:);
end

function [x_full, y_full] = complete_with_homography(x_detected, y_detected, nx, ny)
[ii, jj] = meshgrid(0:nx-1, 0:ny-1); worldPts = [ii(:), jj(:)];
imgPts = [x_detected(:), y_detected(:)];
[tform, ~] = estimateGeometricTransform2D(worldPts, imgPts, ...
    'projective','MaxNumTrials',3000,'Confidence',99.5,'MaxDistance',2.0);
uv_est = transformPointsForward(tform, worldPts);
x_full = uv_est(:,1); y_full = uv_est(:,2);
end

function [xr_aln, yr_aln, mask_has] = align_raw_to_enhanced(x_raw, y_raw, x_enh, y_enh, gate)
% 将 raw 的少量/乱序角点对齐到 enh 的完整索引：
% 对每个 raw 点在 enh 中找最近点（半径 gate 内），唯一分配；输出与 enh 同长度，匹配不上置 NaN。
xr_aln = nan(size(x_enh)); yr_aln = nan(size(y_enh));
mask_has = false(size(x_enh));
if isempty(x_raw) || isempty(y_raw), return; end
Pr = [x_raw(:), y_raw(:)];
Pe = [x_enh(:), y_enh(:)];
D = pdist2(Pr, Pe);              % Nr x Ne
[vals, idx] = min(D,[],2);       % 每个 raw 匹配最近的 enh
used = false(size(Pe,1),1);
for i=1:size(Pr,1)
    j = idx(i);
    if vals(i) <= gate && ~used(j)
        xr_aln(j) = Pr(i,1); yr_aln(j) = Pr(i,2);
        mask_has(j) = true; used(j) = true;
    end
end
end

% ===== 可视化（与原程序一致）=====
function plot_like_legacy(Igray, Xp_abs, Yp_abs, kk, n_sq_y)
PlotCornersX = Yp_abs(:)';   % 列
PlotCornersY = Xp_abs(:)';   % 行
PlotCornerNumber = 1:(numel(PlotCornersX));
figure(2); clf; imagesc(Igray); colormap(gray);
title(['Image ' num2str(kk)]); set(2,'color',[1 1 1]); hold on;
plot(PlotCornersX, PlotCornersY,'+','color','red','linewidth',2);
text(PlotCornersX'+3, PlotCornersY'+3, num2str(PlotCornerNumber')); set(findobj('type','text'),'color','red');
draw_axes(Xp_abs, Yp_abs, n_sq_y);  % 原风格坐标轴
drawnow; hold off;
end

% ===== base 变量工具 =====
function c = assignin_get(name)
try, c = evalin('base',name); catch, c = {}; end
end
function c = set_cell(c, k, v)
if ~iscell(c), c = {}; end
if numel(c) < k, c{k} = []; end
c{k} = v;
end
