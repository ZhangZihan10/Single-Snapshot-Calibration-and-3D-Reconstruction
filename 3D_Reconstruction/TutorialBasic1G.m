%平均算法
%clc
%clear all
% Laser Segmentation
%image = imread('TestImages/image16.jpg');
function output=TutorialBasic1G(a)%对应虚拟模型

Client=a;
image =ImageReadTCP_One(Client,'Center'); %image=imread('image.png');

Cube_l=450;
%image=imresize(image,[1280, 1280]);%imresize(I,[1080, 1080]);更改照片大小
tic;  % 开始计时
img=LaserFind2(image);%新激光识别方法

elapsedTime = toc;  % 结束计时
fprintf('程序运行时间: %.4f 秒\n', elapsedTime);
%img = las_segm(image);%传统激光识别方法
% Mapping
load('Omni_Calib_Results_Unity.mat'); % Calib parameters
ocam_model = calib_data.ocam_model; % Calib parameters
x = 0; % Laser Plane parameters
y = 0; % Laser Plane parameters
las_dist = 4950; % Laser Plane parameters
CVsyst_x = 0; % CV System initial position 在unity中为CVSystemOrigin的位置参数z*1000
CVsyst_y = 0; % CV System initial position 在unity中为CVSystemOrigin的位置参数x*-1000
[x1,y1] =  mapping(img,0,x,y,0,0,0,0,0,las_dist,ocam_model);  % mapping function
% Finally figure:
figure;
scatter(x1,y1,5,'filled'); % Laser intersections
hold on;
plot(CVsyst_x,-CVsyst_y,'r*'); % CV System location
grid on;
% Results validation
% Results validation
% Left Cube
%i=[500;900]; % working image region - column
%j=[100;700]; % working image region - row
%[C_left] = cube_dist(img,i,j,x,y,las_dist,ocam_model);
%C_left = mean(C_left(:,1));
% Up Cube

%识别绿色方块
tic;  % 开始计时
[VX,VY]=ceshiG(image);%识别绿色方块，划定绿色方块所在区域.使用CNN

elapsedTime = toc;  % 结束计时
fprintf('程序运行时间: %.4f 秒\n', elapsedTime);

i=[VX(1);VX(2)]; % working image region - column
j=[VY(1);VY(2)]; % working image region - row
%将数组变为整数
i = round(i);
j = round(j);

[C_Up] = cube_dist(img,i,j,x,y,las_dist,ocam_model);
C_UpMean = mean(C_Up(:,2));%调用y的信息
% Right Cube

[C_Right] = cube_dist(img,i,j,x,y,las_dist,ocam_model);
C_RightMean = mean(C_Right(:,1)); %调用x的信息
%figure(4);
%imshow(image);


%寻找偏移角度
[Y1max,Y1maxRow]=max(C_Up(:,2));%寻找y最大值
X1value = C_Right(Y1maxRow, 1);
[X3max,X3Row]=max(C_Right(:,1));
Y3value = C_Up(X3Row, 2);

[X2min,X2Row]=min(C_Right(:,1));
Y2value = C_Up(X2Row, 2);

test1=C_Up(X2Row+70, 2)-Y1max;%判断物体是否有偏移
if abs(test1)<100
    thetha6=0;
else
   thetha6=90-atan2d(X3max-X1value,Y1max-Y3value);
end


%output=[C_RightMean,C_UpMean, 0];%thetha6];%在计算值基础上y坐标加1厘米
output=[X3max-Cube_l,Y1max-Cube_l,0];
end