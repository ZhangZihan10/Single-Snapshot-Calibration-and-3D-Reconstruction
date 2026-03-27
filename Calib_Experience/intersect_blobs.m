function [Cent, Cent1]=intersect_blobs(img,xx_,zz_)

[height,width] = size(img);  %img=imread('testr14.jpg');
img = im2double(img);

% mask right
xi = [zz_(5);zz_(7);zz_(8);zz_(6)];
yi = [xx_(5);xx_(7);xx_(8);xx_(6)];

%xi = [zz_(8);zz_(7);zz_(6);zz_(5)];
%yi = [xx_(8);xx_(7);xx_(6);xx_(5)];

bw = poly2mask(xi,yi,height,width);
res2 = img.*bw;

figure
imshow(res2);
[Cent] = intersections_v(res2);
% mask left
xi = [zz_(4);zz_(2);zz_(1);zz_(3)];
yi = [xx_(4);xx_(2);xx_(1);xx_(3)];

%xi = [zz_(4);zz_(3);zz_(2);zz_(1)];
%yi = [xx_(4);xx_(3);xx_(2);xx_(1)];
bw = poly2mask(xi,yi,height,width);
res2 = img.*bw;

figure
imshow(res2);
[Cent1] = intersections_v(res2);
end