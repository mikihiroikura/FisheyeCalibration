%%
%��������
clear all
close all
%%
%�摜�f�[�^���܂Ƃ߂�
images = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\old\Laser_off");

%%
%�摜���̃`�F�b�J�[�{�[�h�̓_�����o
[imagePoints,boardSize] = detectCheckerboardPoints(images.Files);

%%
%�`�F�b�J�[�{�[�h�̑傫������_�Q�̃��[���h���W(Z��0)�����
squareSize = 50; % millimeters
CheckboardworldPoints = generateCheckerboardPoints(boardSize,squareSize);

%%
% �`�F�b�J�[�{�[�h�_�CWorld���W�_��Calibration
% ��摜���Ƃ��Ă��ăT�C�Y��}��
I = readimage(images,1); 
imageSize = [size(I,1) size(I,2)];
params = estimateFisheyeParameters(imagePoints,CheckboardworldPoints,imageSize);

%%
% ���[�U�[����̉摜�f�[�^�Q��ǂݎ��
images_on = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\old\Laser_on");

%%
%�_�ɂ����郌�[�U�[�_�̏o�͂�����t���O�̐ݒ�
%0�FOFF�C1:ON
bar_on = 0;

%%
%���ׂẲ摜���烌�[�U�[���ʂ̃J�������W�n�̌v�Z���s��
All_cameraPoints = [];
if bar_on==1
   All_barPixelPoints = []; 
end
for i = 1:size(images_on.Files,1)
    %�摜�̓ǂݍ���
    J = readimage(images_on,i);
    %%%%%%%%Checkerboard�ɂ�����Laser�̏���%%%%%%%%
    %�P�x�d�S�̌v�Z
    [X,Y] = calcCoG(J,0.18,100,300);
    CoGPoints = [X,Y];
    %���[���h���W�n�ɕϊ�
    worldPoints = pointsToWorld(params.Intrinsics,params.RotationMatrices(:,:,i),params.TranslationVectors(i,:),CoGPoints);
    newworldPoints = [worldPoints,zeros(size(worldPoints,1),1)];
    %�J�������W�n�ɕϊ�
    cameraPoints = newworldPoints*params.RotationMatrices(:,:,i)+params.TranslationVectors(i,:);
    All_cameraPoints = [All_cameraPoints;cameraPoints];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if bar_on ==1
    %%%%%%%%%�_�ɂ����郌�[�U�[�_�̏���%%%%%%%%%%%%%%%%%%%%%
        %�P�x�d�S�̌v�Z
        [Xbar,Ybar] = calcCoG(J,0.18,0,10);
        BarPixelPoints = [mean(X),mean(Y)];
        All_barPixelPoints = [All_barPixelPoints;BarPixelPoints];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
end
%%
%���[�U�[����镽��z=ax+by+c�̍ŏ�2�敽�ʂ��v�Z����
Sol = CalcLMSPlane(All_cameraPoints);
%%
%�O��l�̌��o�Ƃ�����Ȃ������ʂ̎��̌v�Z
Optimal_cameraPoints = PlaneOptimization(Sol,All_cameraPoints,3);
Sol2 = CalcLMSPlane(Optimal_cameraPoints);

%%
%���[�U�[���ʂƑS���[�U�[�̃J�������W�_�̃v���b�g
f = figure;
Xcs = All_cameraPoints(:,1);
Ycs = All_cameraPoints(:,2);
Zcs = All_cameraPoints(:,3);
graphX = linspace(min(Xcs),max(Xcs),500);
graphY = linspace(min(Ycs),max(Ycs),500);
[gX,gY] = meshgrid(graphX,graphY);
gZ = Sol(1)*gX+Sol(2)*gY+Sol(3);
mesh(gX,gY,gZ);
hold on
scatter3(Xcs,Ycs,Zcs);
figure
gZ2 = Sol2(1)*gX+Sol2(2)*gY+Sol2(3);
mesh(gX,gY,gZ2,'Facecolor','interp');
hold on
scatter3(Xcs,Ycs,Zcs);
%%
%���x�v���̕]��
Heights = [];
TrueHeights = [];
camOris = [];
erasestr = ["C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_off\picure","mm.jpg"];
for i =1:11%�����11�����̉摜�����x��񎝂��̉摜�Ȃ̂�
    camOri = -params.TranslationVectors(i,:)*params.RotationMatrices(:,:,i)^-1;
    camOris = [camOris;camOri];
    Heights = [Heights;-camOri(3)];
    str = images.Files{i,1};
    TrueHeights = [TrueHeights;str2double(erase(str,erasestr))];
end
diffHeights = Heights - min(Heights);
diffHeights = sort(diffHeights);
diffTrueHeights = TrueHeights- min(TrueHeights);
diffTrueHeights = sort(diffTrueHeights);
%%
%���x���萸�x�O���t�̕\��
f =figure;
no = 0:10;
plot(no,diffTrueHeights,'b-o');
hold on
plot(no,diffHeights,'r-o');
legend('True','Estimate');
g = figure;
plot(no,diffTrueHeights-diffHeights,'g-o');
legend('True-Estimate');
%%
%csv�փJ�����p�����[�^�Q�̏o��
csvfile ='cameraparams.csv';
fid = fopen(csvfile,'w');
fprintf(fid,'%.15f,',params.Intrinsics.MappingCoefficients);
fprintf(fid,'\n');
fprintf(fid,'%f,',params.Intrinsics.StretchMatrix);
fprintf(fid,'\n');
fprintf(fid,'%f,',params.Intrinsics.DistortionCenter);
fprintf(fid,'\n');
for i=1:11
    fprintf(fid,'%f,',TrueHeights(i));
    fprintf(fid,'%f,',params.RotationMatrices(:,:,i));
    fprintf(fid,'\n');
end
%csv�փ��[�U�[���ʃp�����[�^�̏o��
fprintf(fid,'%f,',Sol2);
fprintf(fid,'\n');
%�_�ɉf�郌�[�U�[�_@pixel���W�n�̏o��
if bar_on ==1
    BP = mean(All_barPixelPoints,1);
    fprintf(fid,'%f,',BP);
    fprintf(fid,'\n');
end
fclose(fid);

%%
%���ؒf�@�ɂ�鍂�x�v���̎��H
%�摜�̓ǂݍ���
Ans_worlds = [];
for No = 1:11%������x���邭�Ȃ��ƁC�P�x�d�S���v�Z�ł��Ȃ��ł���
    J = readimage(images_on,No);
    %�P�x�d�S�̌v�Z
    [X,Y] = calcCoG(J);
    CoG_debug = [X,Y];
    %���z�s�N�Z�����W�n�ɕϊ�
    IdealPixs = (CoG_debug-params.Intrinsics.DistortionCenter)*inv(params.Intrinsics.StretchMatrix);
    %�J�������W�n�̒����̎��̖@���x�N�g���̌v�Z
    Lines = [];
    for i = 1:size(IdealPixs,1)
        u = IdealPixs(i,1);
        v = IdealPixs(i,2);
        ro = (u^2+v^2)^0.5;
        w = params.Intrinsics.MappingCoefficients(1)+params.Intrinsics.MappingCoefficients(2)*ro^2+params.Intrinsics.MappingCoefficients(3)*ro^3+params.Intrinsics.MappingCoefficients(4)*ro^4;
        Lines = [Lines;u,v,w];
    end
    %�����̎��ƕ��ʂ���̋���@�J�������W�n
    Ans = [];
    for i = 1:size(Lines,1)
        lamda = -Sol2(3)/(Sol2(1)*Lines(i,1)+Sol2(2)*Lines(i,2)-Lines(i,3));
        Ans = [Ans;lamda*Lines(i,:)];
    end
    if size(Ans,1)==0
        continue
    end
    Ans_world = Ans*inv(params.RotationMatrices(:,:,No));
    Ans_worlds = [Ans_worlds;Ans_world];
end
%���ؒf�@��Calibration���ʂ̍��x�v���̂���
H_btwLS = Ans_worlds(:,3)-All_cameraPoints(:,3);
H_btwLS_outlier = filloutliers(H_btwLS,'linear');
Zure = max(H_btwLS_outlier)-min(H_btwLS_outlier);%�����Ă���_�ł̂���Ȃ̂ŁC���܂�Ӗ��Ȃ��l�H
figure
plotnum =1:size(Ans_worlds,1);
plot(plotnum,H_btwLS);