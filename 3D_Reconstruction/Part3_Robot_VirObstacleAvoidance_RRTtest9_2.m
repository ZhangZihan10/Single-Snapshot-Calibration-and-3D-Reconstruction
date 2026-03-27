%% AGHP-RRT星 路径规划 成功 结合鱼眼相机重构系统结果(真实环境）
clc; clear;

%name = "Matlab";
%Client = TCPInit('127.0.0.1',55016,name);
%arduino=serialport("COM3",115200); %只需要运行1次，连接端口,现在用

% 机械臂参数
gripping_point = 0.056;
L(1) = Link([0 0 0.067 0 1], 'standard'); 
L(2) = Link([0 -0.017 0.092 0 0], 'standard');
L(3) = Link([0 -0.01 0.095 0 0], 'standard');
L(4) = Link([0 -0.04 0 0 0], 'standard');
L(5) = Link([0 0 0 0 0], 'standard');
L(6) = Link([0 0 0 0 0], 'standard');
L(1).qlim = [0.01 0.2];  
L(2).qlim = [-160 160]/180*pi;
L(3).qlim = [-160 160]/180*pi;
L(4).qlim = [0 180]/180*pi;
L(5).qlim = [0 0];  
L(6).qlim = [0 0];  

robot = SerialLink(L, 'name', 'MyRobot');

%% === 输入起点与终点的空间坐标 ===
start_pos = [0.16, 0.08, 0.07]; % 3D坐标
goal_pos  = [0.15, -0.08, 0.05]; % 3D坐标
%%
% === 通过逆解求关节角 ===
q_start = inverse_kinematics(robot, start_pos);
q_goal  = inverse_kinematics(robot, goal_pos);

% 如果逆解失败则退出
if isempty(q_start) || isempty(q_goal)
    error('起点或终点逆解失败，请检查坐标是否超出机械臂工作空间。');
end

% 定义障碍物
%obstacles = {
%    {'sphere', [0.2; 0; 0.1], 0.04} ,{'sphere', [0.2; 0; 0], 0.04} , {'sphere', [0.2; 0.1; 0.07], 0.04}
%};

% === 替换原来的 obstacles 定义 ===
% 假设你已经运行了之前的重构代码，得到了 ptCloud_Recon
%% 3. 自动导入、识别并生成【3D 密集云图】
fig_file = 'image6_test.fig'; 
if ~exist(fig_file, 'file')
    error('未找到文件 %s', fig_file);
end

temp_fig = openfig(fig_file, 'invisible');
% 寻找所有图形对象
target_objs = findobj(temp_fig, 'Type', 'surface');
if isempty(target_objs), target_objs = findobj(temp_fig, 'Type', 'scatter'); end

% --- 在进入循环提取前，必须先初始化变量 ---
X_final = []; 
Y_final = []; 
Z_final = [];
grid_res = 0.01; % 采样间距 2cm

% 3. 遍历识别到的对象并基于角点构建任意角度平面
if ~isempty(target_objs)
    for k = 1:length(target_objs)
        % 提取原始角点数据并缩放
        x_raw = get(target_objs(k), 'XData') / 100;
        y_raw = get(target_objs(k), 'YData') / 100;
        z_raw = get(target_objs(k), 'ZData') / 100;
        
        % 将 2x2 或 1x4 的角点矩阵转换为 4x3 的点列表
        pts = [x_raw(:), y_raw(:), z_raw(:)];
        
        % 只有当点数足以构成面（通常为 4 个角点）时进行面构建
        if size(pts, 1) >= 4
            % 假设角点顺序为：1:左上, 2:左下, 3:右上, 4:右下
            % 定义两个相邻的边缘向量（基向量）
            v_vertical   = pts(2,:) - pts(1,:); % 垂直边 (1 -> 2)
            v_horizontal = pts(3,:) - pts(1,:); % 水平边 (1 -> 3)
            
            % 计算面板的物理尺寸以确定采样点数
            len_v = norm(v_vertical);
            len_h = norm(v_horizontal);
            
            % 生成参数化坐标网格 [0, 1]
            [s, t] = meshgrid(0:(grid_res/len_h):1, 0:(grid_res/len_v):1);
            
            % 使用双线性合成生成平面上的所有点
            % 公式：P(s,t) = P1 + s*v_horizontal + t*v_vertical
            X_grid = pts(1,1) + s*v_horizontal(1) + t*v_vertical(1);
            Y_grid = pts(1,2) + s*v_horizontal(2) + t*v_vertical(2);
            Z_grid = pts(1,3) + s*v_horizontal(3) + t*v_vertical(3);
            
            X_final = [X_final; X_grid(:)];
            Y_final = [Y_final; Y_grid(:)];
            Z_final = [Z_final; Z_grid(:)];
        else
            % 如果点数不足（如离群点），直接拼接
            X_final = [X_final; x_raw(:)];
            Y_final = [Y_final; y_raw(:)];
            Z_final = [Z_final; z_raw(:)];
        end
    end
else
    close(temp_fig); 
    error('未找到 3D 数据，请检查 .fig 文件内容。');
end

close(temp_fig);

% --- 此时运行到第 119 行，变量已存在 ---
valid = ~isnan(X_final) & ~isnan(Y_final) & ~isnan(Z_final);
ptCloud_Recon = pointCloud([X_final(valid), Y_final(valid), Z_final(valid)]);

obstacles = ptCloud_Recon;

%% 1. 基础合法性检查（逆解是否存在）
if isempty(q_start) || isempty(q_goal)
    error('起点或终点逆解失败，请检查坐标是否超出机械臂工作空间。');
end

% 2. 核心修正：初始碰撞检测
% 检测起点是否碰撞
if checkCollision2(robot, q_start, q_start, obstacles)
    
    max_retries = 5; % 每个点尝试 5 次不同的逆解构型
    q_start = [];
    %q_goal = [];
    
    % 搜索合法的起点关节角
    fprintf('正在搜索无碰撞的起点逆解...\n');
    for r = 1:max_retries
        q_try = inverse_kinematics(robot, start_pos);
        if ~isempty(q_try)
            % 检查该构型下机器人是否与点云碰撞
            if ~checkCollision2(robot, q_try, q_try, obstacles)
                q_start = q_try;
                break;
            end
        end
    end
    % 如果最终仍未找到合法解，则退出
    if isempty(q_start)
        error('起点处于点云内部或其所有逆解构型均会碰撞。请移动起点坐标。');
    end
    %error('初始位置 (Start Pos) 发生碰撞！请调整起点坐标。');
end

% 检测目标点是否碰撞
if checkCollision2(robot, q_goal, q_goal, obstacles)
    
    % === 带有碰撞检测的逆解搜索 (避开已碰撞结果) ===
    max_retries = 5; % 每个点尝试 5 次不同的逆解构型
    %q_start = [];
    q_goal = [];
    % 搜索合法的终点关节角
    fprintf('正在搜索无碰撞的终点逆解...\n');
    for r = 1:max_retries
        q_try = inverse_kinematics(robot, goal_pos);
        if ~isempty(q_try)
            if ~checkCollision2(robot, q_try, q_try, obstacles)
                q_goal = q_try;
                break;
            end
        end
    end

    if isempty(q_goal)
        error('终点处于点云内部或其所有逆解构型均会碰撞。请移动终点坐标。');
    end
    %error('目标位置 (Goal Pos) 发生碰撞！请调整终点坐标。');
end

fprintf('起点与终点验证通过，均无碰撞且在可达范围内。\n');

%% PH-RRT参数设置
t = 0.2;         % 均匀概率阈值 (论文推荐0.15~0.3)
rho2_ratio = 0.4; % 目标重力步长比例
rho3_ratio = 0.8; % 随机点步长比例 (ρ2/ρ3=1/2 符合论文)

% 开始计时
fprintf('开始PH-RRT路径规划...\n');
start_time = tic;

% 路径规划
[path, tree] = aghp_rrt_star_planning(robot, q_start, q_goal, obstacles, t, rho2_ratio, rho3_ratio);

% 结束计时
elapsed_time = toc(start_time);
fprintf('路径规划完成，耗时: %.4f 秒\n', elapsed_time);


if ~isempty(path)
    fprintf('路径规划成功，共 %d 个节点\n', size(path,1));
    % 计算关节空间路径长度
    joint_path_length = compute_joint_path_length(path);
    fprintf('关节空间路径长度: %.4f rad\n', joint_path_length);
    % 计算任务空间末端轨迹长度
    cartesian_path_length = compute_cartesian_path_length(robot, path);
    fprintf('任务空间末端轨迹长度: %.4f m\n', cartesian_path_length);

    plot3_path(robot, path, obstacles);
    plot3_rrt_tree(robot, tree, path,false);
    plot3_rrt_robot_poses(robot, path, obstacles);  % 所有节点下机械臂姿态
    
    %b = 1;
    %for a = 1 : length(path)
    %numberTran3(arduino,path(61,1),path(61,2),path(61,3),path(61,4));
    %    b=b+1;     
    %end

else
    disp('路径规划失败');
end

%% ====== AGHP-RRT 核心函数 ======
function [path, tree] = aghp_rrt_star_planning(robot, q_start, q_goal, obstacles, t, rho2_ratio, rho3_ratio, max_iter)
    if nargin < 8
        max_iter = 5000;
    end

    threshold = 0.05;   % 到达目标的阈值
    step_size = 0.05;   % 每次扩展步长
   % dim = 4;            % 有效维度
    grid_resolution = 0.1;  % 任务空间栅格分辨率
    grid_map_xyz = containers.Map('KeyType','char','ValueType','any');

    tree = struct('q', q_start, 'parent', 0, 'cost', 0);
    xyz_start = get_xyz(robot, q_start);
    key_start = get_grid_key_xyz(xyz_start, grid_resolution);
    grid_map_xyz(key_start) = [1];
    open_list = [1];
    found = false;

    for i = 1:max_iter
        open_node_count = length(open_list);
        open_max = 200;
        p = t + (1 - t) * exp(-open_node_count / open_max);
        k = rand();

        if k < p
            q_rand = q_goal;
        else
            q_rand = sample_random_q(robot);
        end

        [q_near, idx_near] = find_nearest_grid(tree, grid_map_xyz, robot, q_rand, grid_resolution);

        if k < p
            direction = normalize_vector(q_goal - q_near);
            q_new = q_near + step_size * direction;
        else
            direction_goal = normalize_vector(q_goal - q_near);
            direction_rand = normalize_vector(q_rand - q_near);
            X1 = rho2_ratio * step_size * direction_goal;
            X2 = rho3_ratio * step_size * direction_rand;
            q_new = q_near + X1 + X2;
        end

        q_new = clamp_to_limits(robot, q_new);

        if ~checkCollision2(robot, q_near, q_new, obstacles)
            [neighbors, neighbor_idxs] = get_neighbors(tree, q_new, grid_map_xyz, robot, grid_resolution, 0.1);
            min_cost = tree(idx_near).cost + norm(q_new - q_near);
            best_parent = idx_near;

            for j = 1:length(neighbors)
                cost_j = tree(neighbor_idxs(j)).cost + norm(q_new - neighbors{j});
                if cost_j < min_cost && ~checkCollision2(robot, neighbors{j}, q_new, obstacles)
                    min_cost = cost_j;
                    best_parent = neighbor_idxs(j);
                end
            end

            tree(end+1) = struct('q', q_new, 'parent', best_parent, 'cost', min_cost);
            new_idx = length(tree);

            xyz_new = get_xyz(robot, q_new);
            key = get_grid_key_xyz(xyz_new, grid_resolution);
            if isKey(grid_map_xyz, key)
                grid_map_xyz(key) = [grid_map_xyz(key), new_idx];
            else
                grid_map_xyz(key) = [new_idx];
            end
            open_list(end+1) = new_idx;

            for j = 1:length(neighbors)
                neighbor = neighbors{j};
                idx = neighbor_idxs(j);
                cost_through_new = min_cost + norm(neighbor - q_new);
                if cost_through_new < tree(idx).cost && ~checkCollision2(robot, q_new, neighbor, obstacles)
                    tree(idx).parent = new_idx;
                    tree(idx).cost = cost_through_new;
                end
            end

            if norm(q_new - q_goal) < threshold && ~checkCollision2(robot, q_new, q_goal, obstacles)
                tree(end+1) = struct('q', q_goal, 'parent', new_idx, 'cost', min_cost + norm(q_new - q_goal));
                found = true;
                break;
            end
        end
    end

    if found
        path = [];
        idx = length(tree);
        while idx ~= 0
            path = [tree(idx).q; path];
            idx = tree(idx).parent;
        end
    else
        path = [];
        warning('AGHP-RRT*未能找到路径');
    end
end




function v = normalize_vector(v)
    if norm(v) > 0
        v = v / norm(v);
    end
end



function [q_near, idx_near] = find_nearest_grid(tree, grid_map_xyz, robot, q_rand, res)
    xyz = get_xyz(robot, q_rand);
    neighbor_keys = generate_neighbor_keys_xyz(xyz, res, 1);
    min_dist = inf;
    idx_near = 1;

    for i = 1:length(neighbor_keys)
        key = neighbor_keys{i};
        if isKey(grid_map_xyz, key)
            idx_list = grid_map_xyz(key);
            for j = 1:length(idx_list)
                q = tree(idx_list(j)).q;
                d = norm(q_rand - q);
                if d < min_dist
                    min_dist = d;
                    idx_near = idx_list(j);
                end
            end
        end
    end
    q_near = tree(idx_near).q;
end


function [neighbors, idxs] = get_neighbors(tree, q_new, grid_map_xyz, robot, res, radius)
    xyz = get_xyz(robot, q_new);
    neighbor_keys = generate_neighbor_keys_xyz(xyz, res, ceil(radius/res));
    neighbors = {};
    idxs = [];
    for i = 1:length(neighbor_keys)
        key = neighbor_keys{i};
        if isKey(grid_map_xyz, key)
            idx_list = grid_map_xyz(key);
            for j = 1:length(idx_list)
                q = tree(idx_list(j)).q;
                if norm(q - q_new) < radius
                    neighbors{end+1} = q;
                    idxs(end+1) = idx_list(j);
                end
            end
        end
    end
end

function xyz = get_xyz(robot, q)
    T = robot.fkine([q 0 0]);
    xyz = T.t';
end

function key = get_grid_key_xyz(xyz, res)
    idx = floor(xyz / res);
    key = sprintf('%d_%d_%d', idx(1), idx(2), idx(3));
end

function keys = generate_neighbor_keys_xyz(xyz, res, range)
    center = floor(xyz / res);
    keys = {};
    for dx = -range:range
        for dy = -range:range
            for dz = -range:range
                idx = center + [dx dy dz];
                key = sprintf('%d_%d_%d', idx(1), idx(2), idx(3));
                keys{end+1} = key;
            end
        end
    end
end

%% ====== 辅助函数 ======
function q = clamp_to_limits(robot, q)
    % 确保关节角度在限位范围内
    for j = 1:4
        qlim = robot.links(j).qlim;
        if q(j) < qlim(1)
            q(j) = qlim(1);
        elseif q(j) > qlim(2)
            q(j) = qlim(2);
        end
    end
end

%% ====== 以下函数保持不变 ======
% 逆解函数（返回4维角度向量）
function q = inverse_kinematics(robot, pos)
    T = transl(pos);       % 目标位姿（无姿态要求）
    % 随机生成一个符合关节限位的初始猜想值 q0
    q0 = sample_random_q(robot);
    try
        %q_full = robot.ikine(T, 'mask', [1 1 1 1 0 0]); % 只考虑位置
        %q = q_full(1:4);    % 只保留前4个关节
        % 传入 q0，让每次搜索的起点都不一样
        q_full = robot.ikine(T, q0, 'mask', [1 1 1 1 0 0]); 
        q = q_full(1:4);
    catch
        q = [];
    end
end

function q = sample_random_q(robot)
    q = zeros(1,4);
    for i = 1:4
        qlim = robot.links(i).qlim;
        q(i) = qlim(1) + rand * (qlim(2) - qlim(1));
    end
end


function collision = checkCollision(robot, q1, q2, obstacles)
    steps = 10;
    for i = 0:steps
        q = q1 + (q2 - q1) * i / steps;
        q_full = [q 0 0];

        % 获取每段连杆的起点与终点位置
        points = zeros(3, 5); % 4连杆 + base
        points(:,1) = [0;0;0];  % 基座点

        for j = 1:4
            Tj = robot.A(1:j, q_full);
            points(:,j+1) = Tj.t;
        end

        % 对每一段连杆（线段）进行检测
        for k = 1:4
            p1 = points(:,k);
            p2 = points(:,k+1);

            for m = 1:length(obstacles)
                obs = obstacles{m};
                if strcmp(obs{1}, 'sphere')
                    center = obs{2};
                    radius = obs{3};

                    % 点到线段距离小于半径 -> 碰撞
                    d = point_to_segment_distance(center, p1, p2);
                    if d < radius
                        collision = true;
                        return;
                    end
                end
            end
        end
    end
    collision = false;
end

function collision = checkCollision2(robot, q1, q2, ptCloud_Obs)
    steps = 20; % 插值步长
    safety_margin = 0.01; % 安全裕量：建议设为 MAE(32mm) 的 1.5 倍左右，即 5cm
    
    for i = 0:steps
        q = q1 + (q2 - q1) * i / steps;
        q_full = [q 0 0];
        
        % 获取机械臂各连杆关节点的 3D 位置
        points = zeros(3, 5); 
        points(:,1) = [0;0;0];  % 基座
        for j = 1:4
            Tj = robot.A(1:j, q_full);
            points(:,j+1) = Tj.t;
        end
        
        % 遍历 4 段连杆
        for k = 1:4
            p1 = points(:,k);
            p2 = points(:,k+1);
            
            % --- 关键：使用点云进行碰撞检测 ---
            % 寻找点云中距离当前连杆线段最近的点
            % 为了加速，我们可以只对连杆中点进行邻域搜索，或采样线段点
            mid_point = (p1 + p2) / 2;
            [~, dists] = knnsearch(ptCloud_Obs.Location, mid_point');
            
            % 如果连杆到点云最近点的距离小于安全半径，判定为碰撞
            if min(dists) < safety_margin
                collision = true;
                return;
            end
        end
    end
    collision = false;
end

function d = point_to_segment_distance(pt, a, b)
    ab = b - a;
    t = dot(pt - a, ab) / dot(ab, ab);
    t = max(0, min(1, t));
    proj = a + t * ab;
    d = norm(pt - proj);
end

function plot3_path(robot, path, obstacles)
    figure('Color', 'w');
    hold on;
    drawObstacles2(obstacles);
    
    view(3);
    grid on;
    axis([-0.3 0.3 -0.3 0.3 0 0.3]); % 按照机械臂尺度设定
    axis equal; % 保持比例，防止变形
    % 用于记录末端执行器轨迹
    trajectory = [];

    for i = 1:size(path,1)
        q = [path(i,:) 0 0];  % 填补无效关节
        robot.plot(q, 'workspace', [-0.3 0.3 -0.3 0.3 -0.3 0.3], 'delay', 0.05);
        T = robot.fkine(q);
        trajectory = [trajectory; T.t'];  % 末端位置加入轨迹
        drawnow;
    end

    % 绘制末端轨迹
    plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), 'b-', 'LineWidth', 2);
    scatter3(trajectory(end,1), trajectory(end,2), trajectory(end,3), 50, 'g', 'filled'); % 终点
    scatter3(trajectory(1,1), trajectory(1,2), trajectory(1,3), 50, 'r', 'filled');       % 起点

    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3);
    title('机械臂运动路径与末端轨迹');
    grid on;
end

function drawObstacles(obstacles)
    for i = 1:length(obstacles)
        obs = obstacles{i};
        switch obs{1}
            case 'sphere'
                center = obs{2};
                radius = obs{3};
                [x, y, z] = sphere(20);
                x = x * radius + center(1);
                y = y * radius + center(2);
                z = z * radius + center(3);
                surf(x, y, z, 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'r');

            case 'cylinder'
                center = obs{2};
                radius = obs{3};
                height = obs{4};
                [x, y, z] = cylinder(radius, 20);
                z = z * height; 
                z = z - height/2 + center(3);
                surf(x + center(1), y + center(2), z, 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'b');
        end
    end
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3);
end

function drawObstacles2(obstacles)
    % 获取当前坐标轴，并强制清理环境
    ax = gca;
    hold(ax, 'on'); 
    
    % --- 设置全局视觉环境：白色背景 ---
    set(gcf, 'Color', 'w');      % 窗口背景设为白色
    set(ax, 'Color', 'w');       % 坐标轴背景设为白色
    set(ax, 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k'); % 刻度设为黑色
    grid(ax, 'on');
    
    if isa(obstacles, 'pointCloud')
        % --- 关键更改：将点云显示颜色设为黄色 [1 1 0] ---
        % 创建一个与点数一致的 RGB 矩阵，每一行都是 [1, 1, 0]
        num_pts = obstacles.Count;
        yellow_color = repmat([1 1 0], num_pts, 1); 
        
        % 绘制黄色点云，并增大 MarkerSize 以便在白背景下看得更清
        pcshow(obstacles.Location, yellow_color, 'MarkerSize', 12, 'Parent', ax);
        
    else
        % --- 几何体障碍物绘制逻辑 (保持不变) ---
        for i = 1:length(obstacles)
            obs = obstacles{i};
            switch obs{1}
                case 'sphere'
                    [x, y, z] = sphere(20);
                    surf(ax, x*obs{3}+obs{2}(1), y*obs{3}+obs{2}(2), z*obs{3}+obs{2}(3), ...
                        'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'r');
                case 'cylinder'
                    [x, y, z] = cylinder(obs{3}, 20);
                    z = z * obs{4} - obs{4}/2 + obs{2}(3);
                    surf(ax, x+obs{2}(1), y+obs{2}(2), z, ...
                        'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'b');
            end
        end
    end
    
    % 视角与缩放优化
    axis(ax, 'equal');
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
    view(ax, 3);
end


function plot3_rrt_tree(robot, tree, path, show_cube)
    if nargin < 4
        show_cube = true;  % 默认显示立方体栅格
    end

    figure;
    hold on;
    view(3);
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('RRT 采样树及任务空间格栅');

    grid_resolution = 0.1;  % 与路径规划中的保持一致
    grid_set = containers.Map();

   

    % ==== 绘制 RRT 树结构 ====
    for i = 2:length(tree)
        q1 = [tree(i).q 0 0];
        q2 = [tree(tree(i).parent).q 0 0];
        T1 = robot.fkine(q1);
        T2 = robot.fkine(q2);
        p1 = T1.t;
        p2 = T2.t;
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'k-', 'LineWidth', 1, 'DisplayName', 'RRT树连接');
    end

    % ==== 绘制最终路径 ====
    if ~isempty(path)
        ee_path = zeros(size(path,1), 3);
        for i = 1:size(path,1)
            q = [path(i,:) 0 0];
            T = robot.fkine(q);
            ee_path(i,:) = T.t';
        end
        plot3(ee_path(:,1), ee_path(:,2), ee_path(:,3), 'r-', 'LineWidth', 3, 'DisplayName', '最佳路径轨迹');
    end

    % ==== 起点终点标记 ====
    T_start = robot.fkine([tree(1).q 0 0]);
    scatter3(T_start.t(1), T_start.t(2), T_start.t(3), 100, 'g', 'filled', 'DisplayName', '起点');
    T_end = robot.fkine([tree(end).q 0 0]);
    scatter3(T_end.t(1), T_end.t(2), T_end.t(3), 100, 'r', 'filled', 'DisplayName', '终点');

    legend('Location','bestoutside');
end


function plot3_rrt_robot_poses(robot, path, obstacles)
    figure;
    hold on;
    grid on;
    view(3);
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('机械臂最终路径姿态及运动轨迹');

    % 绘制障碍物
    drawObstacles2(obstacles);

    % 绘制路径中机械臂各节点姿态（用线条连杆）
    n = size(path,1);
    colors = jet(n); % 用渐变色显示轨迹顺序

    for i = 1:n
         q = [path(i,:) 0 0]; % 补两个无效关节
        % 计算各关节坐标，用forward kinematics分段求各关节位置
        joint_positions = zeros(3, robot.n+1); % 每列一个关节位置 (x,y,z)
        T = eye(4);
        joint_positions(:,1) = T(1:3,4); % 基座坐标原点
        for j = 1:robot.n
            T = T * robot.A(j, q).T;  % 关键修复点
            joint_positions(:, j+1) = T(1:3,4);
        end

        % 绘制连杆
        for j = 1:robot.n
            plot3(joint_positions(1, j:j+1), joint_positions(2, j:j+1), joint_positions(3, j:j+1), ...
                'Color', colors(i,:), 'LineWidth', 2);
        end
    end
    % 计算起点末端执行器位置
    q_start = [path(1,:) 0 0];
    T_start = robot.fkine(q_start);
    pos_start = T_start.t;
    scatter3(pos_start(1), pos_start(2), pos_start(3), 100, 'r', 'filled');
    text(pos_start(1), pos_start(2), pos_start(3), '  Start EE', 'Color', 'g');

    % 计算终点末端执行器位置
    q_end = [path(end,:) 0 0];
    T_end = robot.fkine(q_end);
    pos_end = T_end.t;
    scatter3(pos_end(1), pos_end(2), pos_end(3), 100, 'g', 'filled');
    text(pos_end(1), pos_end(2), pos_end(3), '  Goal EE', 'Color', 'r');

    colorbar;
    colormap(jet);
    caxis([1 n]);
    hold off;
end


function length = compute_joint_path_length(path)
    % 计算关节空间路径长度（各关节角度变化总和）
    length = 0;
    for i = 2:size(path, 1)
        delta = path(i, :) - path(i-1, :);
        length = length + norm(delta);
    end
end


function length = compute_cartesian_path_length(robot, path)
    % 计算任务空间末端轨迹长度
    length = 0;
    prev_pos = [];
    
    for i = 1:size(path, 1)
        q_full = [path(i, :) 0 0];
        T = robot.fkine(q_full);
        current_pos = T.t';
        
        if ~isempty(prev_pos)
            delta = current_pos - prev_pos;
            length = length + norm(delta);
        end
        
        prev_pos = current_pos;
    end
end



