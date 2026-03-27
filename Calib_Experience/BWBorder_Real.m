%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%   Author: Ivan Kholodilin
%   email: kholodilin@bit.edu.cn
%   website: www.ilabit.org
%
%   Copyright (C) 2020 Ivan Kholodilin
%   
%   This program is free software; you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation; either version 2 of the License, or
%   (at your option) any later version.
%   
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%   
%   You should have received a copy of the GNU General Public License
%   along with this program; if not, write to the Free Software
%   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
%   USA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function BWEdge = BWBorder_Real(Image)

% detect region of black pixels   Image=imread('testr30_2_1.jpg');
Black = Image(:,:,1)<=100 & Image(:,:,2)<=100 & Image(:,:,3)<=100;   %imshow(Black)
%Black = Image(:,:,1)>=20 & Image(:,:,1)<=80 & Image(:,:,2)>=20 & Image(:,:,2)<=80 & Image(:,:,3)>=20 & Image(:,:,3)<=80; 
% detect region of white pixels
White = Image(:,:,1)>=200 & Image(:,:,2)>=200 & Image(:,:,3)>=200;   %imshow(White)
[h,w] = size(Image(:,:,1)); % image size

Obj =Black; 
BlackEdge = edge(Obj,'sobel'); % black border detection  %imshow(BlackEdge)
[x,y] = find(BlackEdge==1); % endpoints detection
BWEdge(1:h,1:w) = 0; % border between Black & White   %imshow(BWEdge)

for t=1:length(x)
    if(x(t)==1 || y(t)==1 || y(t)==w || x(t)==h)
        r=0;
    elseif(x(t)-3<1 || y(t)-3<1 || y(t)+3>w || x(t)+3>h)
        r=1;
    else
        r = 3;
    end
    mask = White(x(t)-r:x(t)+r,y(t)-r:y(t)+r); % check 3x3 region
    [s1,s2] = find(mask==1);
    % if there's a white region mark as a border between Black & White
    if(length(s1)~=0)   
        BWEdge(x(t),y(t)) = 1;
    end
    
    clear s1;
    clear s2;
end
% figure;
%BWEdge = bwmorph(BWEdge,'skel',Inf);
%BWEdge = bwareaopen(BWEdge,120);
% 已经是骨架
BWEdge = bwmorph(BWEdge,'skel',Inf);

% 1. 极轻微膨胀（只影响近邻）
BWEdge = imdilate(BWEdge, strel('disk',5));

% 2. 连接近距离断点
BWEdge = bwmorph(BWEdge,'bridge',Inf);

% 3. 再细化回单像素
BWEdge = bwmorph(BWEdge,'skel',Inf);

%BWEdge = bwmorph(BWEdge,'skel',Inf);
stats = regionprops(bwconncomp(BWEdge,18),'Area');
BWEdge=ismember(labelmatrix(bwconncomp(BWEdge,8)), find([stats.Area] > 120));
BWEdge = bwmorph(BWEdge,'bridge',Inf);   %imshow(BWEdge)
end