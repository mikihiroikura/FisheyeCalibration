%%
%��������
clear all
close all
% %%
% %Laser�Ȃ��̉摜�f�[�^1�̕ۑ�
% images = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_off1");
% %Laser����̉摜�f�[�^�Q�P�̌Ăяo��
% images_on = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_on1");
% %%
% %Laser�Ȃ��̉摜�f�[�^2�̕ۑ�
% images2 = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_off2");
% %Laser����̉摜�f�[�^�Q�Q�̌Ăяo��
% images_on2 = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_on2");
% %%
% %Laser�Ȃ��̉摜�f�[�^3�̕ۑ�
% images3 = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_off3");
% %Laser����̉摜�f�[�^�Q3�̌Ăяo��
% images_on3 = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_on3");
% 
% %%
% %Laser�Ȃ��̉摜�f�[�^4�̕ۑ�
% images4 = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_off4");
% %Laser����̉摜�f�[�^�Q4�̌Ăяo��
% images_on4 = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_on4");

%%
%�摜�Q����̔z��Ɋi�[����
calibnum = 10;
all_image_off = {};
all_image_on = {};
rootdir_off = "C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_off";
rootdir_on = "C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_on";
for i = 1:calibnum
    dir_off = strcat(rootdir_off,num2str(i));
    dir_on = strcat(rootdir_on,num2str(i));
    image_off = imageDatastore(dir_off);
    image_on = imageDatastore(dir_on);
    all_image_off{i} = image_off;
    all_image_on{i} = image_on;
end

%%
%�����郌�[�U�[�͈̔͂̎w��
laser = [150,300];
bar = [385,400];

%%
% %���ׂĂ�Laser_off�摜��Cameracalinration
% camparam = All_Loff_cameraCalib(all_image_off);
% disp('end')
%%
% %calibration���ʂƂ��ꂼ���Laser_on�摜�ŕ��ʂ̎��Ɩ_��̓_�Q@camera���W
% all_plane = [];
% all_barpoints = [];
% all_laserpoint = [];
% lasercnt = [];
% cnt = 0;
% for i = 1:size(all_image_on,2)
%     [plane,laserpoint,barpoint] = CameraPlaneBarCalibration(camparam,all_image_on{i},laser,bar,0.4,150.0/255.0,cnt);
%     all_plane = [all_plane;plane];
%     all_barpoints = [all_barpoints;barpoint];
%     all_laserpoint = [all_laserpoint;laserpoint];
%     lasercnt = [lasercnt;size(laserpoint,1)];
%     cnt = cnt + size(all_image_off,2);
% end
%%
% %calibration�Ƃ��ꂼ���Laser_on�摜�ŕ��ʂ̎��Ɩ_��̓_�Q@camera���W
% all_plane = [];
% all_barpoints = [];
% all_laserpoint = [];
% lasercnt = [];
% params = {};
% for i = 1:size(all_image_on,2)
%     [param,plane,laserpoint,barpoint] = CameraPlaneBarCalibration_each(all_image_off{i},all_image_on{i},laser,bar,0.4,150.0/255.0);
%     all_plane = [all_plane;plane];
%     all_barpoints = [all_barpoints;barpoint];
%     all_laserpoint = [all_laserpoint;laserpoint];
%     lasercnt = [lasercnt;size(laserpoint,1)];
%     params{i} = param;
%     X = sprintf('%d th calibration ends.',i);
%     disp(X);
% end

%%
%�܂�1�Z�b�g��Laser_off�摜�Q�ŃJ�����p�����[�^�̎擾
param = Loff_cameraCalib(all_image_off{1});
%%
%�J�����p�����[�^�͋��ʂɂ��ĕ��ʂ̎����v�Z����
all_plane = [];
all_barpoints = [];
all_laserpoint =[];
lasercnt = [];
for i=1:size(all_image_on,2)
    [plane,laserpoint,barpoint] = CameraPlaneBarCalibration(param,all_image_on{i},laser,bar,0.4,150.0/255.0,0);
    all_plane = [all_plane;plane];
    all_barpoints = [all_barpoints;barpoint];
    all_laserpoint = [all_laserpoint;laserpoint];
    lasercnt = [lasercnt;size(laserpoint,1)];
    X = sprintf('%d th calibration ends.',i);
    disp(X);
end
%%
%�_�ɂ����郌�[�U�[�_�Q����_�㕽��z=ax+by+c�̍ŏ�2�敽�ʂ��v�Z����
barSol = CalcLMSPlane(all_barpoints);
%�O��l�̌��o�Ƃ�����Ȃ������ʂ̎��̌v�Z
Optimal_barpoints = PlaneOptimization(barSol,all_barpoints,3);
baroptSol = CalcLMSPlane(Optimal_barpoints);

%%
%3���ʂ���ł��߂��_(�P�_)�̌v�Z
point = MultiPlane2Point(all_plane);
%%
%�v�Z���ʂ̕ۑ�
csvfile ='cameramultiplanebarparams.csv';
fid = fopen(csvfile,'w');
%Camera�p�����[�^�P
fprintf(fid,'%.15f,',param.Intrinsics.MappingCoefficients);
fprintf(fid,'\n');
fprintf(fid,'%f,',param.Intrinsics.StretchMatrix);
fprintf(fid,'\n');
fprintf(fid,'%f,',param.Intrinsics.DistortionCenter);
fprintf(fid,'\n');
fprintf(fid,'%f,',param.RotationMatrices(:,:,1));
fprintf(fid,'\n');
%�_��̕��ʎ�z=ax+by+c
fprintf(fid,'%f,',baroptSol);
fprintf(fid,'\n');
%�P�_�̍��W
fprintf(fid,'%f,',point);
fprintf(fid,'\n');
fclose(fid);
%%
%���ʁCBar���ʁC�P�_�̕`��
f = figure;
cnt = 0;
for i = 1:calibnum
    X = all_laserpoint(1+cnt:cnt+lasercnt(i),1);
    Y = all_laserpoint(1+cnt:cnt+lasercnt(i),2);
    Z = all_laserpoint(1+cnt:cnt+lasercnt(i),3);
    scatter3(X,Y,Z);
    hold on
    graphX = linspace(min(X),max(X),500);
    graphY = linspace(min(Y),max(Y),500);
    [gX,gY] = meshgrid(graphX,graphY);
    gZ = all_plane(i,1)*gX+all_plane(i,2)*gY+all_plane(i,3);
    mesh(gX,gY,gZ);
    cnt = cnt + lasercnt(i);
end
%�_���ʂ̕`��
B = [all_barpoints(:,1),all_barpoints(:,2),all_barpoints(:,3)];
scatter3(B(:,1),B(:,2),B(:,3));
graphX = linspace(min(B(:,1)),max(B(:,1)),500);
graphY = linspace(min(B(:,2)),max(B(:,2)),500);
[gX,gY] = meshgrid(graphX,graphY);
gZ = baroptSol(1)*gX+baroptSol(2)*gY+baroptSol(3);
mesh(gX,gY,gZ);
%�P�_�̕`��
scatter3(point(1),point(2),point(3),'o');
%%
%�摜�o�̓f�o�b�N
imshow(string(images_on.Files(4)));
impixelinfo;
%%
%�P�_�ƕ��ʂƂ̋����̌v�Z
for x = 1:10
    dist = abs(all_plane(x,1)*point(1)+all_plane(x,2)*point(2)-point(3)+all_plane(x,3))/(all_plane(x,1)^2+all_plane(x,2)^2+1)^0.5;
    disp(dist);
end