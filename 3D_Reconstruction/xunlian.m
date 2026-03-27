clc
clear;
load gTruth.mat;
imageDir='测试图片\';
imds=imageDatastore(imageDir);

classNames=["red", "blue","green","background"];
pxds=pixelLabelDatastore(gTruth);

imageSize=[1080 1080 3];
numClasses=numel(classNames);
lgraph=deeplabv3plusLayers(imageSize,numClasses,"resnet18");

pximds=pixelLabelImageDatastore(imds,pxds,'OutputSize',[1080 1080 3],...
    'ColorPreprocessing','gray2rgb');

opts=trainingOptions("sgdm",'ExecutionEnvironment','gpu',...
    'InitialLearnRate',0.001,'MiniBatchSize',2,'Plots',...
    'training-progress','MaxEpochs',20);

[net,info]=trainNetwork(pximds,lgraph,opts);

save('trainedNet.mat','net');
save('trainedInfo.mat','info');
% 加载验证数据

