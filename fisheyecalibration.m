%%
%�摜�f�[�^���܂Ƃ߂�
images = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\HighSpeedCamera\sample\PhotoCapture\Photos\Laser_off");

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
images_on = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\HighSpeedCamera\sample\PhotoCapture\Photos\Laser_on");

%%
%���ׂẲ摜���烌�[�U�[���ʂ̃J�������W�n�̌v�Z���s��
All_cameraPoints = [];
for i = 1:size(images_on.Files,1)
    %�摜�̓ǂݍ���
    J = readimage(images_on,i);
    %�P�x�d�S�̌v�Z
    [X,Y] = calcCoG(J);
    CoGPoints = [X,Y];
    %���[���h���W�n�ɕϊ�
    worldPoints = pointsToWorld(params.Intrinsics,params.RotationMatrices(:,:,i),params.TranslationVectors(i,:),CoGPoints);
    newworldPoints = [worldPoints,zeros(size(worldPoints,1),1)];
    %�J�������W�n�ɕϊ�
    cameraPoints = newworldPoints*params.RotationMatrices(:,:,i)+params.TranslationVectors(i,:);
    All_cameraPoints = [All_cameraPoints;cameraPoints];
end
%%
%���[�U�[����镽��z=ax+by+c�̍ŏ�2�敽�ʂ��v�Z����
Xcs = All_cameraPoints(:,1);
Ycs = All_cameraPoints(:,2);
Zcs = All_cameraPoints(:,3);
Sx2 = sum(Xcs.*Xcs);
Sy2 = sum(Ycs.*Ycs);
Sxy = sum(Xcs.*Ycs);
Sx = sum(Xcs);
Sy = sum(Ycs);
Sz = sum(Zcs);
Sxz = sum(Xcs.*Zcs);
Syz = sum(Ycs.*Zcs);
N = size(Xcs,1);
A = [Sx2,Sxy,Sx;...
     Sxy,Sy2,Sy;...
     Sx,Sy,N];
b = [Sxz;Syz;Sz];
Sol = A\b;

%%
%���[�U�[���ʂƑS���[�U�[�̃J�������W�_�̃v���b�g
f = figure;
graphX = linspace(min(Xcs),max(Xcs),500);
graphY = linspace(min(Ycs),max(Ycs),500);
[gX,gY] = meshgrid(graphX,graphY);
gZ = Sol(1)*gX+Sol(2)*gY+Sol(3);
mesh(gX,gY,gZ);
hold on
scatter3(Xcs,Ycs,Zcs);
%%
%���x�v���̕]��
Heights = [];
TrueHeights = [];
erasestr = ["C:\Users\Mikihiro Ikura\Documents\GitHub\HighSpeedCamera\sample\PhotoCapture\Photos\Laser_off\picure","mm.jpg"];
for i =1:15%�����15�����̉摜�����x��񎝂��̉摜�Ȃ̂�
    camOri = -params.TranslationVectors(i,:)*params.RotationMatrices(:,:,i)^-1;
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
no = 0:14;
plot(no,diffTrueHeights,'b-o');
hold on
plot(no,diffHeights,'r-o');
legend('True','Estimate');
g = figure;
plot(no,diffTrueHeights-diffHeights,'g-o');
legend('True-Estimate');
