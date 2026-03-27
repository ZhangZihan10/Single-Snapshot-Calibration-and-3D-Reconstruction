%clear all;

% 获取基本名称
basename = input('Basename camera calibration images (without number nor suffix): ', 's');

% 获取图像格式
valid_formats = {'r', 'ras', 'b', 'bmp', 't', 'tif', 'g', 'gif', 'p', 'pgm', 'j', 'jpg', 'm', 'ppm'};
format_extensions = {'ras', 'ras', 'bmp', 'bmp', 'tif', 'tif', 'gif', 'gif', 'pgm', 'pgm', 'jpg', 'jpg', 'ppm', 'ppm'};

format_input = input('Image format: ([]=''r''=''ras'', ''b''=''bmp'', ''t''=''tif'', ''g''=''gif'', ''p''=''pgm'', ''j''=''jpg'', ''m''=''ppm''): ', 's');

format_idx = find(strcmpi(valid_formats, format_input), 1);
if isempty(format_idx)
    fprintf('Invalid format, use default format jpg\n');
    format_idx = 11; % 'j'的索引
end
file_ext = format_extensions{format_idx};

% 获取图像数量
num_images = input('The number of images to be processed: ');

% 获取起始编号
start_num = input('Start number: ');
img_numbers = start_num:(start_num + num_images - 1);

% 预分配内存
im_h = cell(num_images, 1);

% 处理图像
for i = 1:num_images
    img_number = img_numbers(i);
    img_name = sprintf('%s%d.%s', basename, img_number, file_ext);
    
    try
        % 读取图像
        im = imread(img_name);
        fprintf('Processing image: %s\n', img_name);
    catch
        fprintf('Error: Unable to read image% s, please check if the file exists. \n', img_name);
        continue; % 跳过当前图像，继续处理下一个
    end
    
    
    % 设置参数
    up_scale = 3;
    model = 'model\9-5-5(ImageNet)\x3.mat';


    % 仅处理亮度分量（如果是彩色图像）
    if size(im, 3) > 1
        im = rgb2ycbcr(im);
        im = im(:, :, 1);
    end 
    
    
    % 对图像进行裁剪，使图像尺寸能被放大倍数整除
    im_gnd = modcrop(im, up_scale);
    % 将图像数据转换为单精度类型，并归一化到 [0, 1] 区间
    im_gnd = single(im_gnd) / 255;

    % 进行双三次插值
    im_l = imresize(im_gnd, 1 / up_scale, 'bicubic');
    im_b = imresize(im_l, up_scale, 'bicubic');

    % 使用 SRCNN 模型进行超分辨率重建
    im_h{i} = SRCNN(model, im_b);  %i=1;

    % 保存图像
    output_name = sprintf('%s_%d.jpg', basename, img_number);
    imwrite(im_h{i}, output_name);

    %--- Step 6: 显示对比（可选） ---
    figure;
    subplot(1,3,1); imshow(im, []); title('原始高分辨率');
    subplot(1,3,2); imshow(im_b, []); title('双三次插值');
    subplot(1,3,3); imshow(im_h{i}, []); title('SRCNN 重建结果');

end

 
fprintf('\nAll image processing completed!\n');