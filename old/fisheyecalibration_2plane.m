%%
%消去する
clear all
close all
%%
%Laserなしの画像データの保存
images = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_off1");

%%
%Laserありの画像データ群１の呼び出し
images_on = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_on1");
%%
%Laserなしの画像データ2の保存
images2 = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_off2");

%%
%Laserありの画像データ群２の呼び出し
images_on2 = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_on2");

%%
%Camera calibration
%レーザーなしのCheckerboard画像群から求める
%画像中からChekerboard点の検出
[imagePoints,boardSize] = detectCheckerboardPoints(images.Files);
squareSize = 50; % millimeters
CheckboardworldPoints = generateCheckerboardPoints(boardSize,squareSize);
I = readimage(images,1); 
imageSize = [size(I,1) size(I,2)];
%calibration結果の保存
params = estimateFisheyeParameters(imagePoints,CheckboardworldPoints,imageSize);


%%
%棒に映る輝点群の格納
All_barPixelPoints = [];
%%
All_barCameraPoints = [];
%%
%レーザー画像群１から平面計算
%Camera座標系でのレーザー点群の保存
%Pixel座標系での棒に映るレーザー点の保存
All_cameraPoints = [];
for i = 1:size(images_on.Files,1)
    %画像の読み込み
    J = readimage(images_on,i);
    %%%%%%%%CheckerboardにあたるLaserの処理%%%%%%%%
    %輝度重心の計算
    [X,Y] = calcCoG(J,0.25,150,300);
    CoGPoints = [X,Y];
    %ワールド座標系に変換
    worldPoints = pointsToWorld(params.Intrinsics,params.RotationMatrices(:,:,i),params.TranslationVectors(i,:),CoGPoints);
    newworldPoints = [worldPoints,zeros(size(worldPoints,1),1)];
    %カメラ座標系に変換
    cameraPoints = newworldPoints*params.RotationMatrices(:,:,i)+params.TranslationVectors(i,:);
    All_cameraPoints = [All_cameraPoints;cameraPoints];
    %%%%%%%%%棒にあたるレーザー点の処理%%%%%%%%%%%%%%%%%%%%%
    %輝度重心の計算
    [Xbar,Ybar] = calcCoG(J,0.3,389,435);
    BarPixelPoints = [mean(Xbar),mean(Ybar)];
    All_barPixelPoints = [All_barPixelPoints;BarPixelPoints];
end
%レーザー点群から最小2乗平面を計算する
%レーザーが作る平面z=ax+by+cの最小2乗平面を計算する
Sol = CalcLMSPlane(All_cameraPoints);
%外れ値の検出とそれを省いた平面の式の計算
Optimal_cameraPoints = PlaneOptimization(Sol,All_cameraPoints,3);
Sol_opt = CalcLMSPlane(Optimal_cameraPoints);
%%
%2つ目のCamera calibration
%レーザーなしのCheckerboard画像群から求める
%画像中からChekerboard点の検出
[imagePoints,boardSize] = detectCheckerboardPoints(images2.Files);
squareSize = 50; % millimeters
CheckboardworldPoints = generateCheckerboardPoints(boardSize,squareSize);
I = readimage(images2,1); 
imageSize = [size(I,1) size(I,2)];
%calibration結果の保存
params2 = estimateFisheyeParameters(imagePoints,CheckboardworldPoints,imageSize);


%%
%レーザー画像群2から平面計算
%Camera座標系でのレーザー点群の保存
All_barPixelPoints2 = [];
All_cameraPoints2 = [];
for i = 1:size(images_on2.Files,1)
    %画像の読み込み
    J = readimage(images_on2,i);
    %%%%%%%%CheckerboardにあたるLaserの処理%%%%%%%%
    %輝度重心の計算
    [X,Y] = calcCoG(J,0.4,150,300);
    CoGPoints = [X,Y];
    %ワールド座標系に変換
    worldPoints = pointsToWorld(params2.Intrinsics,params2.RotationMatrices(:,:,i),params2.TranslationVectors(i,:),CoGPoints);
    newworldPoints = [worldPoints,zeros(size(worldPoints,1),1)];
    %カメラ座標系に変換
    cameraPoints = newworldPoints*params2.RotationMatrices(:,:,i)+params2.TranslationVectors(i,:);
    All_cameraPoints2 = [All_cameraPoints2;cameraPoints];
    %輝度重心の計算
    [Xbar,Ybar] = calcCoG(J,0.3,389,435);
    BarPixelPoints = [mean(Xbar),mean(Ybar)];
    All_barPixelPoints2 = [All_barPixelPoints2;BarPixelPoints];
end
%レーザー点群から最小2乗平面を計算する
%レーザーが作る平面z=ax+by+cの最小2乗平面を計算する
Sol2 = CalcLMSPlane(All_cameraPoints2);
%外れ値の検出とそれを省いた平面の式の計算
Optimal_cameraPoints2 = PlaneOptimization(Sol2,All_cameraPoints2,3);
Sol_opt2 = CalcLMSPlane(Optimal_cameraPoints2);
%%
%2つの平面から直線の式の方向ベクトルと，通過点を求める
dirVect = [1;-(Sol_opt(1)-Sol_opt2(1))/(Sol_opt(2)-Sol_opt2(2));...
(Sol_opt2(1)*Sol_opt(2)-Sol_opt(1)*Sol_opt2(2))/(Sol_opt(2)-Sol_opt2(2))];
planePoint = [0, -(Sol_opt(3)-Sol_opt2(3))/(Sol_opt(2)-Sol_opt2(2)),...
    (Sol_opt(2)*Sol_opt2(3)-Sol_opt2(2)*Sol_opt(3))/(Sol_opt(2)-Sol_opt2(2))];

%%
%レーザー画像群に映る棒の上の輝点群をカメラ座標系に移す
%点群から直線の式を最小2乗で求める
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
%PCAを用いた最適直線のあてはめ
[coeff,score,roots] = pca(All_barCameraPoints);
dirVect2 = coeff(:,1);
meanbarCam = mean(All_barCameraPoints,1);
%dievec:直線の方向ベクトル，meanbarcam:通る点
% %外れ値の検出とそれを除いた直線を求める
Optimal_barCamPoints = LineOptimization(dirVect2,meanbarCam,All_barCameraPoints,10);
[coeff,score,roots] = pca(Optimal_barCamPoints);
dirVect2 = coeff(:,1);
meanbarCam = mean(All_barCameraPoints,1);
%%
%csvへカメラパラメータ，2平面共通直線，棒の直線の出力
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
%csvへ2平面共通直線の出力
fprintf(fid,'%f,',dirVect);
fprintf(fid,'\n');
fprintf(fid,'%f,',planePoint);
fprintf(fid,'\n');
%棒の直線式の出力
fprintf(fid,'%f,',dirVect2);
fprintf(fid,'\n');
fprintf(fid,'%f,',meanbarCam);
fprintf(fid,'\n');
fclose(fid);
%%
% %画像出力のデバック
% X = readimage(images2,1);
% imshow(X)
% impixelinfo;

%%
%2平面，平面共通直線，棒直線の可視化
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