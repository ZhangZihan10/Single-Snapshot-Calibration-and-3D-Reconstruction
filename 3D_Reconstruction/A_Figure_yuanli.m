%% 绘制原理图
clear; clc; close all;

% 创建画布
figure('Color', 'white', 'Name', '鱼眼球面映射原理 (Code Coordinate System)');
axes_handle = axes;
hold on; axis equal; grid on;
xlabel('X (水平)'); ylabel('Y (主光轴/深度)'); zlabel('Z (垂直)');
view(135, 25); % 调整视角以便观察
xlim([-1.5 2.5]); ylim([-0.5 3]); zlim([-1.5 1.5]);

%% 1. 绘制 CCD 传感器平面 (X-Z 平面)
% 在你的代码中，鱼眼图像位于 X-Z 平面上 (Y=0)
t = linspace(0, 2*pi, 100);
r_sensor = 1.2;
fill3(r_sensor*cos(t), zeros(size(t)), r_sensor*sin(t), ...
    [0.8 0.8 0.8], 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineStyle', '--');
text(1.3, 0, 0, 'CCD传感器 (鱼眼图像)', 'FontSize', 10);

%% 2. 绘制单位球面 (半球)
% 你的光轴是 Y 轴，所以半球面向 Y 正方向
[sx, sy, sz] = sphere(50);
sy(sy < 0) = nan; % 只保留 Y > 0 的部分
surf(sx, sy, sz, 'FaceColor', 'cyan', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
% 画出光轴
quiver3(0,0,0, 0,2.5,0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5); 
text(0, 2.6, 0, '光轴 (Y)', 'Color', 'g', 'FontWeight', 'bold');

%% 3. 模拟代码中的“虚拟贴图平面” (Target Plane)
% 代码：P = [u, 1, v]，即在 Y=2 (假设距离) 处有一个点
dist = 2.0;
% 假设障碍物上的一点 P_world
P_world = [0.8, dist, 1.2]; % 任意取一个点
plot3(P_world(1), P_world(2), P_world(3), 'ms', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
text(P_world(1), P_world(2), P_world(3)+0.2, '虚拟障碍物上的点 P_{world}', 'Color', 'm');

%% 4. 绘制光线 (Ray)
plot3([0, P_world(1)], [0, P_world(2)], [0, P_world(3)], 'r-', 'LineWidth', 1.5);

%% 5. 计算球面交点 (Spherical Mapping)
% 这是代码中: phi 和 theta 的物理意义
vec = P_world / norm(P_world); % 单位向量
P_sphere = vec; 

% 在球面上画出该点
plot3(P_sphere(1), P_sphere(2), P_sphere(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
text(P_sphere(1)+0.1, P_sphere(2), P_sphere(3), '球面映射点', 'Color', 'r', 'FontSize', 8);

%% 6. 绘制投影到 CCD 的过程 (等距投影)
% phi: 光线与 Y 轴夹角
phi = atan2(norm([P_world(1), P_world(3)]), P_world(2));
% theta: 在 X-Z 平面上的投影角
theta = atan2(P_world(3), P_world(1));

% 模拟成像点 (r 正比于 phi)
f_sim = 1.0; % 模拟焦距
r_img = f_sim * phi; 
img_x = r_img * cos(theta);
img_z = r_img * sin(theta);

% 画出 CCD 上的成像点
plot3(img_x, 0, img_z, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
text(img_x, 0, img_z-0.2, '像素点 P_{pixel}', 'Color', 'b');

% 连线示意投影关系 (虚线)
plot3([P_sphere(1), img_x], [P_sphere(2), 0], [P_sphere(3), img_z], 'k:', 'LineWidth', 1);

%% 7. 标注角度 (关键部分)
% 画 Phi (入射角)
arc_r = 0.5;
t_phi = linspace(0, phi, 20);
% 在包含光线和Y轴的平面内画弧
v_axis = [0,1,0]; % Y轴
v_ray = vec;
axis_rot = cross(v_axis, v_ray); axis_rot = axis_rot/norm(axis_rot);
for k=1:length(t_phi)
    R_mat = axang2rotm([axis_rot, t_phi(k)]);
    pt = R_mat * [0; arc_r; 0];
    plot3(pt(1), pt(2), pt(3), 'k-');
end
text(0, 0.6, 0.1, '\phi (入射角)', 'FontSize', 12);

% 画 Theta (方位角) - 在 X-Z 平面上
arc_r_theta = 0.4;
t_theta = linspace(0, theta, 20);
plot3(arc_r_theta*cos(t_theta), zeros(size(t_theta)), arc_r_theta*sin(t_theta), 'b-');
text(0.5, 0, 0.2, '\theta (方位角)', 'Color', 'b');

title('代码逻辑对应的球面映射原理图');