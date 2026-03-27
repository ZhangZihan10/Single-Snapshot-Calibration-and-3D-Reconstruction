% === 回调函数 ===
function updateHeight(src, ~)
    % 从figure的UserData中提取变量
    fig = ancestor(src, 'figure');
    W = fig.UserData.W;
    srcPoints = fig.UserData.srcPoints;
    correctedImg = fig.UserData.correctedImg;
    
    % 动态更新参数
    currentH = src.Value;
    dstPoints = [0, 0; W, 0; W, currentH; 0, currentH];
    
    % 计算单应性变换
    H = estimateGeometricTransform(srcPoints, dstPoints, 'projective');
    orthoImg = imwarp(correctedImg, H);
    
    % 更新图像显示
    imshow(orthoImg, 'Parent', gca);
end


