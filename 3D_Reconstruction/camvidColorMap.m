function cmap = camvidColorMap()
% 修改颜色映射，匹配你的 6 个类别
cmap = [
    0     0   255 % 蓝色 - bluecube
    0   255   0   % 绿色 - greencube
    255 165   0   % 橙色 - orangecube
    %255 255   0   % 黄色 - yellowsqhere
    %255   0   0   % 红色 - redsqhere
    %255 255 255   % 白色 - background
]; 
%cmap = [
    %255   0   0   % 红色 - redsqhere
    %0     0   255 % 蓝色 - bluecube
    %0   255   0   % 绿色 - greencube
    %255 255   0   % 黄色 - yellowsqhere
    %255 165   0   % 橙色 - orangecube
    %255 255 255   % 白色 - background
%]; 
cmap = cmap ./ 255; % 将 0-255 范围归一化到 0-1
end