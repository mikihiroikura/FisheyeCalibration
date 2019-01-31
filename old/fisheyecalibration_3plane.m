%%
%消去する
clear all
close all
% %%
% %Laserなしの画像データ1の保存
% images = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_off1");
% %Laserありの画像データ群１の呼び出し
% images_on = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_on1");
% %%
% %Laserなしの画像データ2の保存
% images2 = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_off2");
% %Laserありの画像データ群２の呼び出し
% images_on2 = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_on2");
% %%
% %Laserなしの画像データ3の保存
% images3 = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_off3");
% %Laserありの画像データ群3の呼び出し
% images_on3 = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_on3");
% 
% %%
% %Laserなしの画像データ4の保存
% images4 = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_off4");
% %Laserありの画像データ群4の呼び出し
% images_on4 = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_on4");

%%
%画像群を一つの配列に格納する
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
%あたるレーザーの範囲の指定
laser = [150,300];
bar = [385,400];

%%
% %すべてのLaser_off画像でCameracalinration
% camparam = All_Loff_cameraCalib(all_image_off);
% disp('end')
%%
% %calibration結果とそれぞれのLaser_on画像で平面の式と棒上の点群@camera座標
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
% %calibrationとそれぞれのLaser_on画像で平面の式と棒上の点群@camera座標
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
%まず1セットのLaser_off画像群でカメラパラメータの取得
param = Loff_cameraCalib(all_image_off{1});
%%
%カメラパラメータは共通にして平面の式を計算する
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
%棒にあたるレーザー点群から棒上平面z=ax+by+cの最小2乗平面を計算する
barSol = CalcLMSPlane(all_barpoints);
%外れ値の検出とそれを省いた平面の式の計算
Optimal_barpoints = PlaneOptimization(barSol,all_barpoints,3);
baroptSol = CalcLMSPlane(Optimal_barpoints);

%%
%3平面から最も近い点(輝点)の計算
point = MultiPlane2Point(all_plane);
%%
%計算結果の保存
csvfile ='cameramultiplanebarparams.csv';
fid = fopen(csvfile,'w');
%Cameraパラメータ１
fprintf(fid,'%.15f,',param.Intrinsics.MappingCoefficients);
fprintf(fid,'\n');
fprintf(fid,'%f,',param.Intrinsics.StretchMatrix);
fprintf(fid,'\n');
fprintf(fid,'%f,',param.Intrinsics.DistortionCenter);
fprintf(fid,'\n');
fprintf(fid,'%f,',param.RotationMatrices(:,:,1));
fprintf(fid,'\n');
%棒上の平面式z=ax+by+c
fprintf(fid,'%f,',baroptSol);
fprintf(fid,'\n');
%輝点の座標
fprintf(fid,'%f,',point);
fprintf(fid,'\n');
fclose(fid);
%%
%平面，Bar平面，輝点の描画
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
%棒平面の描画
B = [all_barpoints(:,1),all_barpoints(:,2),all_barpoints(:,3)];
scatter3(B(:,1),B(:,2),B(:,3));
graphX = linspace(min(B(:,1)),max(B(:,1)),500);
graphY = linspace(min(B(:,2)),max(B(:,2)),500);
[gX,gY] = meshgrid(graphX,graphY);
gZ = baroptSol(1)*gX+baroptSol(2)*gY+baroptSol(3);
mesh(gX,gY,gZ);
%輝点の描画
scatter3(point(1),point(2),point(3),'o');
%%
%画像出力デバック
imshow(string(images_on.Files(4)));
impixelinfo;
%%
%輝点と平面との距離の計算
for x = 1:10
    dist = abs(all_plane(x,1)*point(1)+all_plane(x,2)*point(2)-point(3)+all_plane(x,3))/(all_plane(x,1)^2+all_plane(x,2)^2+1)^0.5;
    disp(dist);
end