%%RRT-Connect计算，成功

clc; clear;

% 机械臂参数
gripping_point = 0.056;
L(1) = Link([0 0 0.067 0 1], 'standard'); 
L(2) = Link([0 -0.017 0.092 0 0], 'standard');
L(3) = Link([0 -0.01 0.095 0 0], 'standard');
L(4) = Link([0 -0.04 0 0 0], 'standard');
L(5) = Link([0 0 0 0 0], 'standard');
L(6) = Link([0 0 0 0 0], 'standard');
L(1).qlim = [0.01 0.14];  
L(2).qlim = [-160 160]/180*pi;
L(3).qlim = [-160 160]/180*pi;
L(4).qlim = [0 180]/180*pi;
L(5).qlim = [0 0];  
L(6).qlim = [0 0];  

robot = SerialLink(L, 'name', 'MyRobot');

%% === 输入起点与终点的空间坐标 ===
start_pos = [0.15, -0.06, 0.03]; % 3D坐标
goal_pos  = [0.17, 0.07, 0.05]; % 3D坐标
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
    fprintf('初始起点构型碰撞，尝试切换构型...\n');
    found_safe = false;
    
    q_candidates = ikine_scara_analytic(robot, start_pos);

    if isempty(q_candidates)
        error('起点超出机械臂物理工作空间！');
    end
    
    found_safe = false;
    for r = 1:size(q_candidates, 1)
        q_try = q_candidates(r, :);
        % 检查当前构型（整根连杆）是否碰撞
        if ~checkCollision2(robot, q_try, q_try, obstacles)
            q_start = q_try;
            found_safe = true;
            fprintf('成功找到安全起点构型：解编号 %d\n', r);
            break;
        end
    end
    
    if ~found_safe
        error('起点位置正确，但“左手系”和“右手系”两种姿态均会碰撞障碍物。');
    end
   
end

% 检测目标点是否碰撞
% 2. 核心修正：初始碰撞检测
% 检测起点是否碰撞
if checkCollision2(robot, q_goal, q_goal, obstacles)
    fprintf('目标点构型碰撞，尝试切换构型...\n');
    found_safe = false;
    
    q_candidates = ikine_scara_analytic(robot, goal_pos);

    if isempty(q_candidates)
        error('目标超出机械臂物理工作空间！');
    end
    
    found_safe = false;
    for r = 1:size(q_candidates, 1)
        q_try = q_candidates(r, :);
        % 检查当前构型（整根连杆）是否碰撞
        if ~checkCollision2(robot, q_try, q_try, obstacles)
            q_goal = q_try;
            found_safe = true;
            fprintf('成功找到安全目标点构型：解编号 %d\n', r);
            break;
        end
    end
    
    if ~found_safe
        error('起点位置正确，但“左手系”和“右手系”两种姿态均会碰撞障碍物。');
    end
   
end

%numberTran3(arduino,q_start(1,1),q_start(1,2),q_start(1,3),q_start(1,4));%将计算结果转换为电机参数,并传入arduino

% 开始计时
fprintf('开始RRT-Connect路径规划...\n');
start_time = tic;

% 路径规划
[path, treeA, treeB] = rrt_connect_path_planning(robot, q_start, q_goal, obstacles);

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
    plot3_rrt_tree(robot, [treeA, treeB], path);
    plot3_rrt_robot_poses(robot, path, obstacles);
else
    disp('路径规划失败');
end

%% ====== 工具函数 ======

% 逆解函数（返回4维角度向量）
function q = inverse_kinematics(robot, pos)
    T = transl(pos);       % 目标位姿（无姿态要求）
    try
        q_full = robot.ikine(T, 'mask', [1 1 1 1 0 0]); % 只考虑位置
        q = q_full(1:4);    % 只保留前4个关节
    catch
        q = [];
    end
end

function [path, treeA, treeB] = rrt_connect_path_planning(robot, q_start, q_goal, obstacles, max_iter)
    if nargin < 5
        max_iter = 5000;
    end

    threshold = 0.05;  
    step_size = 0.05;
    dim = 4;

    treeA = struct('q', q_start, 'parent', 0);  % 从起点出发的树
    treeB = struct('q', q_goal, 'parent', 0);   % 从终点出发的树（反向）

    for iter = 1:max_iter
        if mod(iter,2) == 1
            [success, treeA, treeB, connect_idx_A, connect_idx_B] = extend_rrt_connect(robot, treeA, treeB, step_size, threshold, obstacles);
        else
            [success, treeB, treeA, connect_idx_B, connect_idx_A] = extend_rrt_connect(robot, treeB, treeA, step_size, threshold, obstacles);
        end

        if success
            % 回溯路径
            pathA = backtrack_path(treeA, connect_idx_A);
            pathB = backtrack_path(treeB, connect_idx_B);
            pathB = flipud(pathB);  % 反转从 goal 到连接点的路径

            path = [pathA; pathB];
            return;
        end
    end

    path = [];
    warning('RRT-Connect 未能找到路径');
end

function [success, tree1, tree2, idx1, idx2] = extend_rrt_connect(robot, tree1, tree2, step_size, threshold, obstacles)
    q_rand = sample_random_q(robot);
    [q_near, idx_near] = find_nearest(tree1, q_rand);
    q_new = steer(q_near, q_rand, step_size);

    if ~checkCollision2(robot, q_near, q_new, obstacles)
        tree1(end+1) = struct('q', q_new, 'parent', idx_near);
        q_connect = q_new;

        % 尝试将 tree2 延伸到 q_connect
        [q_near2, idx_near2] = find_nearest(tree2, q_connect);
        q_new2 = steer(q_near2, q_connect, step_size);

        while ~checkCollision2(robot, q_near2, q_new2, obstacles)
            tree2(end+1) = struct('q', q_new2, 'parent', idx_near2);
            if norm(q_new2 - q_connect) < threshold
                success = true;
                idx1 = length(tree1);
                idx2 = length(tree2);
                return;
            end
            idx_near2 = length(tree2);
            q_near2 = q_new2;
            q_new2 = steer(q_near2, q_connect, step_size);
        end
    end
    success = false;
    idx1 = -1;
    idx2 = -1;
end

function path = backtrack_path(tree, idx)
    path = [];
    while idx ~= 0
        path = [tree(idx).q; path];
        idx = tree(idx).parent;
    end
end

function q = sample_random_q(robot)
    q = zeros(1,4);
    for i = 1:4
        qlim = robot.links(i).qlim;
        q(i) = qlim(1) + rand * (qlim(2) - qlim(1));
    end
end

function [q_near, idx] = find_nearest(tree, q_rand)
    min_dist = inf;
    idx = 1;
    for i = 1:length(tree)
        d = norm(tree(i).q - q_rand);
        if d < min_dist
            min_dist = d;
            idx = i;
        end
    end
    q_near = tree(idx).q;
end

function q_new = steer(q1, q2, step_size)
    direction = q2 - q1;
    dist = norm(direction);
    if dist <= step_size
        q_new = q2;
    else
        q_new = q1 + (direction / dist) * step_size;
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
    steps = 15; % 插值步长（关节空间移动的平滑度）
    link_samples = 8; % 关键改进：在每根连杆上均匀取 8 个点进行检测
    
    % 根据你机械臂的实际粗细设定（单位：米）
    % 建议设为：连杆物理半径 + 视觉误差裕量(约0.02)
    link_radius = 0.04; 

    for i = 0:steps
        q = q1 + (q2 - q1) * i / steps;
        q_full = [q 0 0];
        
        % 获取各关节点坐标
        points = zeros(3, 5); 
        points(:,1) = [0;0;0];  
        for j = 1:4
            Tj = robot.A(1:j, q_full);
            points(:,j+1) = Tj.t;
        end
        
        % 遍历 4 段连杆
        for k = 1:4
            p_start = points(:,k);
            p_end = points(:,k+1);
            
            % --- 改进点：对整根连杆进行多点采样 ---
            for s = 0:(1/link_samples):1
                test_point = p_start + s * (p_end - p_start);
                
                % 搜索点云中距离当前采样点最近的点
                [~, dists] = knnsearch(ptCloud_Obs.Location, test_point');
                
                % 如果距离小于连杆半径，判定碰撞
                if min(dists) < link_radius
                    collision = true;
                    return;
                end
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
    figure;
    hold on;
    drawObstacles(obstacles);

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

function plot3_rrt_tree(robot, tree, path)
    figure;
    hold on;
    view(3);
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('RRT 采样树及最终路径');

    % 画出 RRT 树结构
    for i = 2:length(tree)
         parent_idx = tree(i).parent;
        if parent_idx == 0
            continue;  % 根节点，无需画线
        end
        q1 = [tree(i).q 0 0];
    q2 = [tree(parent_idx).q 0 0];
    T1 = robot.fkine(q1);
    T2 = robot.fkine(q2);
    p1 = T1.t;
    p2 = T2.t;
    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'k-', 'LineWidth', 1);
    end

    % 高亮显示最佳路径（末端执行器轨迹）
    if ~isempty(path)
        ee_path = zeros(size(path,1), 3); % 每行一个末端执行器的位置
        for i = 1:size(path,1)
            q = [path(i,:) 0 0];
            T = robot.fkine(q);
            ee_path(i,:) = T.t';
        end
        plot3(ee_path(:,1), ee_path(:,2), ee_path(:,3), 'r-', 'LineWidth', 3); % 红色粗线表示最佳路径
    end

    % 起点和终点标记
    T_start = robot.fkine([tree(1).q 0 0]);
    scatter3(T_start.t(1), T_start.t(2), T_start.t(3), 60, 'r', 'filled');  % 起点红色
    T_end = robot.fkine([tree(end).q 0 0]);
    scatter3(T_end.t(1), T_end.t(2), T_end.t(3), 60, 'g', 'filled');        % 终点绿色

    legend('RRT树连接', '最佳路径轨迹', '起点', '终点');
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
    drawObstacles(obstacles);

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

function q_all = ikine_scara_analytic(robot, pos)
    % pos: [x, y, z]
    % q_all: 2x4 矩阵，每一行是一个解
    x = pos(1)-0.07; y = pos(2); z = pos(3);
    
    % 提取连杆长度 (a1 对应 L2, a2 对应 L3)
    a1 = robot.links(2).a; 
    a2 = robot.links(3).a;
    
    % 计算关节 1 (Z 轴) - SCARA 的 Z 轴通常是直接对应或线性关系
    % 根据你的 Link 定义：L(1) 是移动关节，偏移量为 0.067
    q1 = z +0.067; 
    if q1<0.01 || q1>0.14
        error("计算错误");
    end
    
    % 计算关节 2 和 3 的几何关系
    D = (x^2 + y^2 - a1^2 - a2^2) / (2 * a1 * a2);
    
    % 检查是否超出工作空间
    if abs(D) > 1
        q_all = []; return; 
    end
    
    % 解 1：右手系 (Elbow Down)
    q3_1 = acos(D);
    q2_1 = atan2(y, x) - atan2(a2*sin(q3_1), a1 + a2*cos(q3_1));
    
    % 解 2：左手系 (Elbow Up)
    q3_2 = -acos(D);
    q2_2 = atan2(y, x) - atan2(a2*sin(q3_2), a1 + a2*cos(q3_2));
    
    % 拼装结果 [q1, q2, q3, q4] (假设 q4 暂时给 0)
    q_all = [q1, q2_1, q3_1, 0;
             q1, q2_2, q3_2, 0];
end


