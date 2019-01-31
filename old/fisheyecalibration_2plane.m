%%
%��������
clear all
close all
%%
%Laser�Ȃ��̉摜�f�[�^�̕ۑ�
images = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_off1");

%%
%Laser����̉摜�f�[�^�Q�P�̌Ăяo��
images_on = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_on1");
%%
%Laser�Ȃ��̉摜�f�[�^2�̕ۑ�
images2 = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_off2");

%%
%Laser����̉摜�f�[�^�Q�Q�̌Ăяo��
images_on2 = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_on2");

%%
%Camera calibration
%���[�U�[�Ȃ���Checkerboard�摜�Q���狁�߂�
%�摜������Chekerboard�_�̌��o
[imagePoints,boardSize] = detectCheckerboardPoints(images.Files);
squareSize = 50; % millimeters
CheckboardworldPoints = generateCheckerboardPoints(boardSize,squareSize);
I = readimage(images,1); 
imageSize = [size(I,1) size(I,2)];
%calibration���ʂ̕ۑ�
params = estimateFisheyeParameters(imagePoints,CheckboardworldPoints,imageSize);


%%
%�_�ɉf��P�_�Q�̊i�[
All_barPixelPoints = [];
%%
All_barCameraPoints = [];
%%
%���[�U�[�摜�Q�P���畽�ʌv�Z
%Camera���W�n�ł̃��[�U�[�_�Q�̕ۑ�
%Pixel���W�n�ł̖_�ɉf�郌�[�U�[�_�̕ۑ�
All_cameraPoints = [];
for i = 1:size(images_on.Files,1)
    %�摜�̓ǂݍ���
    J = readimage(images_on,i);
    %%%%%%%%Checkerboard�ɂ�����Laser�̏���%%%%%%%%
    %�P�x�d�S�̌v�Z
    [X,Y] = calcCoG(J,0.25,150,300);
    CoGPoints = [X,Y];
    %���[���h���W�n�ɕϊ�
    worldPoints = pointsToWorld(params.Intrinsics,params.RotationMatrices(:,:,i),params.TranslationVectors(i,:),CoGPoints);
    newworldPoints = [worldPoints,zeros(size(worldPoints,1),1)];
    %�J�������W�n�ɕϊ�
    cameraPoints = newworldPoints*params.RotationMatrices(:,:,i)+params.TranslationVectors(i,:);
    All_cameraPoints = [All_cameraPoints;cameraPoints];
    %%%%%%%%%�_�ɂ����郌�[�U�[�_�̏���%%%%%%%%%%%%%%%%%%%%%
    %�P�x�d�S�̌v�Z
    [Xbar,Ybar] = calcCoG(J,0.3,389,435);
    BarPixelPoints = [mean(Xbar),mean(Ybar)];
    All_barPixelPoints = [All_barPixelPoints;BarPixelPoints];
end
%���[�U�[�_�Q����ŏ�2�敽�ʂ��v�Z����
%���[�U�[����镽��z=ax+by+c�̍ŏ�2�敽�ʂ��v�Z����
Sol = CalcLMSPlane(All_cameraPoints);
%�O��l�̌��o�Ƃ�����Ȃ������ʂ̎��̌v�Z
Optimal_cameraPoints = PlaneOptimization(Sol,All_cameraPoints,3);
Sol_opt = CalcLMSPlane(Optimal_cameraPoints);
%%
%2�ڂ�Camera calibration
%���[�U�[�Ȃ���Checkerboard�摜�Q���狁�߂�
%�摜������Chekerboard�_�̌��o
[imagePoints,boardSize] = detectCheckerboardPoints(images2.Files);
squareSize = 50; % millimeters
CheckboardworldPoints = generateCheckerboardPoints(boardSize,squareSize);
I = readimage(images2,1); 
imageSize = [size(I,1) size(I,2)];
%calibration���ʂ̕ۑ�
params2 = estimateFisheyeParameters(imagePoints,CheckboardworldPoints,imageSize);


%%
%���[�U�[�摜�Q2���畽�ʌv�Z
%Camera���W�n�ł̃��[�U�[�_�Q�̕ۑ�
All_barPixelPoints2 = [];
All_cameraPoints2 = [];
for i = 1:size(images_on2.Files,1)
    %�摜�̓ǂݍ���
    J = readimage(images_on2,i);
    %%%%%%%%Checkerboard�ɂ�����Laser�̏���%%%%%%%%
    %�P�x�d�S�̌v�Z
    [X,Y] = calcCoG(J,0.4,150,300);
    CoGPoints = [X,Y];
    %���[���h���W�n�ɕϊ�
    worldPoints = pointsToWorld(params2.Intrinsics,params2.RotationMatrices(:,:,i),params2.TranslationVectors(i,:),CoGPoints);
    newworldPoints = [worldPoints,zeros(size(worldPoints,1),1)];
    %�J�������W�n�ɕϊ�
    cameraPoints = newworldPoints*params2.RotationMatrices(:,:,i)+params2.TranslationVectors(i,:);
    All_cameraPoints2 = [All_cameraPoints2;cameraPoints];
    %�P�x�d�S�̌v�Z
    [Xbar,Ybar] = calcCoG(J,0.3,389,435);
    BarPixelPoints = [mean(Xbar),mean(Ybar)];
    All_barPixelPoints2 = [All_barPixelPoints2;BarPixelPoints];
end
%���[�U�[�_�Q����ŏ�2�敽�ʂ��v�Z����
%���[�U�[����镽��z=ax+by+c�̍ŏ�2�敽�ʂ��v�Z����
Sol2 = CalcLMSPlane(All_cameraPoints2);
%�O��l�̌��o�Ƃ�����Ȃ������ʂ̎��̌v�Z
Optimal_cameraPoints2 = PlaneOptimization(Sol2,All_cameraPoints2,3);
Sol_opt2 = CalcLMSPlane(Optimal_cameraPoints2);
%%
%2�̕��ʂ��璼���̎��̕����x�N�g���ƁC�ʉߓ_�����߂�
dirVect = [1;-(Sol_opt(1)-Sol_opt2(1))/(Sol_opt(2)-Sol_opt2(2));...
(Sol_opt2(1)*Sol_opt(2)-Sol_opt(1)*Sol_opt2(2))/(Sol_opt(2)-Sol_opt2(2))];
planePoint = [0, -(Sol_opt(3)-Sol_opt2(3))/(Sol_opt(2)-Sol_opt2(2)),...
    (Sol_opt(2)*Sol_opt2(3)-Sol_opt2(2)*Sol_opt(3))/(Sol_opt(2)-Sol_opt2(2))];

%%
%���[�U�[�摜�Q�ɉf��_�̏�̋P�_�Q���J�������W�n�Ɉڂ�
%�_�Q���璼���̎����ŏ�2��ŋ��߂�
for i=1:size(All_barPixelPoints,1)
    Line = Pixel2CameraRay(All_barPixelPoints(i,:),params);
    lamda = -Sol_opt(3)/(Sol_opt(1)*Line(1)+Sol_opt(2)*Line(2)-Line(3));
    All_barCameraPoints = [All_barCameraPoints;lamda*Line];
end
for i=1:size(All_barPixelPoints2,1)
    Line = Pixel2CameraRay(All_barPixelPoints2(i,:),params2);
    lamda = -Sol_opt2(3)/(Sol_opt2(1)*Line(1)+Sol_opt2(2)*Line(2)-Line(3));
    All_barCameraPoints = [All_barCameraPoints;lamda*Line];
end
%%
%PCA��p�����œK�����̂��Ă͂�
[coeff,score,roots] = pca(All_barCameraPoints);
dirVect2 = coeff(:,1);
meanbarCam = mean(All_barCameraPoints,1);
%dievec:�����̕����x�N�g���Cmeanbarcam:�ʂ�_
% %�O��l�̌��o�Ƃ�������������������߂�
Optimal_barCamPoints = LineOptimization(dirVect2,meanbarCam,All_barCameraPoints,10);
[coeff,score,roots] = pca(Optimal_barCamPoints);
dirVect2 = coeff(:,1);
meanbarCam = mean(All_barCameraPoints,1);
%%
%csv�փJ�����p�����[�^�C2���ʋ��ʒ����C�_�̒����̏o��
csvfile ='cameraplaneparams.csv';
fid = fopen(csvfile,'w');
fprintf(fid,'%.15f,',params.Intrinsics.MappingCoefficients);
fprintf(fid,'\n');
fprintf(fid,'%f,',params.Intrinsics.StretchMatrix);
fprintf(fid,'\n');
fprintf(fid,'%f,',params.Intrinsics.DistortionCenter);
fprintf(fid,'\n');
fprintf(fid,'%f,',params.RotationMatrices(:,:,1));
fprintf(fid,'\n');
%csv��2���ʋ��ʒ����̏o��
fprintf(fid,'%f,',dirVect);
fprintf(fid,'\n');
fprintf(fid,'%f,',planePoint);
fprintf(fid,'\n');
%�_�̒������̏o��
fprintf(fid,'%f,',dirVect2);
fprintf(fid,'\n');
fprintf(fid,'%f,',meanbarCam);
fprintf(fid,'\n');
fclose(fid);
%%
% %�摜�o�͂̃f�o�b�N
% X = readimage(images2,1);
% imshow(X)
% impixelinfo;

%%
%2���ʁC���ʋ��ʒ����C�_�����̉���
f = figure;
Xcs = All_cameraPoints(:,1);
Ycs = All_cameraPoints(:,2);
Zcs = All_cameraPoints(:,3);
scatter3(Xcs,Ycs,Zcs);
hold on
graphX = linspace(min(Xcs),max(Xcs),500);
graphY = linspace(min(Ycs),max(Ycs),500);
[gX,gY] = meshgrid(graphX,graphY);
gZ = Sol_opt(1)*gX+Sol_opt(2)*gY+Sol_opt(3);
mesh(gX,gY,gZ);
Xcs2 = All_cameraPoints2(:,1);
Ycs2 = All_cameraPoints2(:,2);
Zcs2 = All_cameraPoints2(:,3);
scatter3(Xcs2,Ycs2,Zcs2);
hold on
graphX2 = linspace(min(Xcs2),max(Xcs2),500);
graphY2 = linspace(min(Ycs2),max(Ycs2),500);
[gX2,gY2] = meshgrid(graphX2,graphY2);
gZ2 = Sol_opt2(1)*gX2+Sol_opt2(2)*gY2+Sol_opt2(3);
mesh(gX2,gY2,gZ2);
t = [min(score(:,1))-.2, max(score(:,1))+.2];
endpts = [meanbarCam + t(1)*dirVect2'; meanbarCam + t(2)*dirVect2'];
plot3(endpts(:,1),endpts(:,2),endpts(:,3),'k-');
maxY = max(max(max(Ycs),max(Ycs2)),max(endpts(:,2)));
minY = min(min(min(Ycs),min(Ycs2)),min(endpts(:,2)));
t2 = [-10000 10000];
endpts2 = [planePoint+t2(1)*dirVect';planePoint+t2(2)*dirVect'];
plot3(endpts2(:,1),endpts2(:,2),endpts2(:,3),'r-');
ylim([minY maxY]);
view(dirVect);