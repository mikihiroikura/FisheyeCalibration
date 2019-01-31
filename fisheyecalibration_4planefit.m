%%
clear all
close all

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
%�܂�1�Z�b�g��Laser_off�摜�Q�ŃJ�����p�����[�^�̎擾
param = Loff_cameraCalib(all_image_off{1});

%%
%�J�����p�����[�^�͋��ʂɂ��ĕ��ʂ̎����v�Z����
%���ʂ̎��͂��̎��_��ax+by+cz=1�ɂȂ��Ă���
all_plane = [];
all_barpoints = [];
all_laserpoint =[];
lasercnt = [];
barX = [];
for i=1:size(all_image_on,2)
    [plane,laserpoint,barpoint,barpixelpoints] = CameraPlaneBarCalibration2(param,all_image_on{i},laser,bar,0.4,150.0/255.0,0);
    all_plane = [all_plane;plane];
    all_barpoints = [all_barpoints;barpoint];
    barX = [barX;mean(barpixelpoints(:,1))];
    all_laserpoint = [all_laserpoint;laserpoint];
    lasercnt = [lasercnt;size(laserpoint,1)];
    X = sprintf('%d th calibration ends.',i);
    disp(X);
end
%%
%��ԃO���t�̕`��
barXp = linspace(min(barX),max(barX));
%�O���t�̕⊮
pca = pchip(barX,all_plane(:,1),barXp);
pcb = pchip(barX,all_plane(:,2),barXp);
pcc = pchip(barX,all_plane(:,3),barXp);
%�⊮�����O���t�̉�A
pa = polyfit(barXp,pca,4);
pb = polyfit(barXp,pcb,4);
pc = polyfit(barXp,pcc,4);
ya = polyval(pa,barXp);
yb = polyval(pb,barXp);
yc = polyval(pc,barXp);
%%
%�}�ɏo��
%���ʃp�����[�^���
figure
plot(barX,all_plane(:,1),'o',barXp,pca,'or',barXp,ya);
figure
plot(barX,all_plane(:,2),'o',barXp,pcb,'or',barXp,yb);
figure
plot(barX,all_plane(:,3),'o',barXp,pcc,'or',barXp,yc);
%���[�U�[���ʂ̕`��
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
    gZ = -all_plane(i,1)/all_plane(i,3)*gX-all_plane(i,2)/all_plane(i,3)*gY+1/all_plane(i,3);
    mesh(gX,gY,gZ);
    cnt = cnt + lasercnt(i);
end
%%
%�v�Z���ʂ̕ۑ�
csvfile ='cameraplanebarparamsinterpolate.csv';
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
%�_���Laser�Q�̕��X���W
fprintf(fid,'%f,',barXp);
fprintf(fid,'\n');
%Y0 = 0,Z0 = 200�ł�X���W���
fprintf(fid,'%f,',pcpoint);
fprintf(fid,'\n');
%ax+by+cz=1�̃p�����[�^
fprintf(fid,'%f,',pca);
fprintf(fid,'\n');
fprintf(fid,'%f,',pcb);
fprintf(fid,'\n');
fprintf(fid,'%f,',pcc);
fprintf(fid,'\n');
fclose(fid);