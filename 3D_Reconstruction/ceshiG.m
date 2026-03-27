function [VBX,VBY]=ceshiG(img)
load trainedNet3.mat;%trainedNet1为resnet18模型，trainedNet3为resnet50模型
%[file,path]=uigetfile('D:\桌面文件夹\robot course\arduino\视觉识别\语义分割虚拟\测试图片\');
%filepath=fullfile(path,file);
I=img;


%cam=webcam(2);
%preview(cam);
%cam.Resolution='1920x1080';
%cam.Brightness=-10;%调整相机亮度
%I =snapshot(cam);

figure;
imshow(I);

I=imresize(I,[1280, 1280]);%imresize(I,[1080, 1080]);

C=semanticseg(I,net,'MiniBatchSize', 32);
%pxds =pixelLabelDatastore(I,classes,labelIDs);
%classes=["green","red", "blue","background"];%["red", "blue","green","background"];
%classes=["Bei","Red", "Green","Black","Grey"];
%cmap=camvidColorMap;%需要更改内参数
%B=labeloverlay(I,C,'ColorMap',cmap,'Transparency',0.4);
%figure;
%imshow(B),title("Semantic segmentation Result");
%pixelLabelColorbar(cmap,classes);

%寻找绿色方块
C1=cellstr(C);
LB = [];
for i =10:1250 %1:size(C1, 1)
    for j = 10:1250  %1:size(C1, 2)
        if strcmp(C1(i, j), "green")%选定检测对象
            LB = [LB; i, j];
        end
    end
end

%黑色方块中心点
meanValueB = mean(LB, 1);
%图片尺寸信息为列*行
Valuex=[round(meanValueB(2)*1.5)-70,round(meanValueB(2)*1.5)+70];%确定区域范围,由于虚拟照片与真实照片间尺寸有区别，所有转换尺寸
Valuey=[round(meanValueB(1)*1.5)-70,round(meanValueB(1)*1.5)+70];
VBX=Valuex;
VBY=Valuey;
end