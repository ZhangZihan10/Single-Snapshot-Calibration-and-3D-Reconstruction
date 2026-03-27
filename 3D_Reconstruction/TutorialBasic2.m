%寻找最小值算法
%clc
%clear all
% Laser Segmentation
%image = imread('TestImages/image16.jpg');
function output=TutorialBasic2(a)%对应虚拟模型
Cube_l=450;%900约等于2.26cm，像素数×0.002517为实际长度cm
% Laser Segmentation
%name = "Matlab";
%Client = TCPInit('127.0.0.1',55012,name);
Client=a;
image =ImageReadTCP_One(Client,'Center'); %image=imread('image.png');

img = las_segm(image);
% Mapping
load('Omni_Calib_Results_Unity.mat'); % Calib parameters
ocam_model = calib_data.ocam_model; % Calib parameters
x = 0; % Laser Plane parameters
y = 0; % Laser Plane parameters
las_dist = 950; % Laser Plane parameters
CVsyst_x = 0; % CV System initial position 在unity中为CVSystemOrigin的位置参数z*1000
CVsyst_y = 0; % CV System initial position 在unity中为CVSystemOrigin的位置参数x*-1000
[x1,y1] = mapping(img,x,y,las_dist,ocam_model); % mapping function
% Finally figure:
figure(3);
scatter(x1,y1,5,'filled'); % Laser intersections
hold on;
plot(CVsyst_x,-CVsyst_y,'r*'); % CV System location
grid on;
% Results validation
% Results validation
% Left Cube
%i=[20;850]; % working image region - column
%j=[10;700]; % working image region - row
%[C_left] = cube_dist(img,i,j,x,y,las_dist,ocam_model);
%C_left = mean(C_left(:,1));
% Up Cube
i=[1;900];%i=[850;1250]; % working image region - column
j=[500;1900];%j=[10;700]; % working image region - row
[C_Up] = cube_dist(img,i,j,x,y,las_dist,ocam_model);
C_Up1 = min(C_Up(:,2));%调用y的信息
%C_Up = mean(C_Up(:,2));%调用y的信息
% Right Cube
i=[1;900];%i=[850;1250]; % working image region - column
j=[500;1900];%j=[10;700]; % working image region - row
[C_Right] = cube_dist(img,i,j,x,y,las_dist,ocam_model);
C_Right1 = min(C_Right(:,1)); %调用x的信息
%C_Right = mean(C_Right(:,1)); %调用x的信息
figure(4);
imshow(image);
output=[C_Right1+Cube_l,C_Up1+Cube_l];%在计算值基础上y坐标加1厘米
end