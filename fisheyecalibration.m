%%
%消去する
clear all
close all
%%
%画像データをまとめる
images = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_off");

%%
%画像中のチェッカーボードの点を検出
[imagePoints,boardSize] = detectCheckerboardPoints(images.Files);

%%
%チェッカーボードの大きさから点群のワールド座標(Zは0)を取る
squareSize = 50; % millimeters
CheckboardworldPoints = generateCheckerboardPoints(boardSize,squareSize);

%%
% チェッカーボード点，World座標点でCalibration
% 一つ画像をとってきてサイズを図る
I = readimage(images,1); 
imageSize = [size(I,1) size(I,2)];
params = estimateFisheyeParameters(imagePoints,CheckboardworldPoints,imageSize);

%%
% レーザーありの画像データ群を読み取る
images_on = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_on");

%%
%すべての画像からレーザー平面のカメラ座標系の計算を行う
All_cameraPoints = [];
for i = 1:size(images_on.Files,1)
    %画像の読み込み
    J = readimage(images_on,i);
    %輝度重心の計算
    [X,Y] = calcCoG(J);
    CoGPoints = [X,Y];
    %ワールド座標系に変換
    worldPoints = pointsToWorld(params.Intrinsics,params.RotationMatrices(:,:,i),params.TranslationVectors(i,:),CoGPoints);
    newworldPoints = [worldPoints,zeros(size(worldPoints,1),1)];
    %カメラ座標系に変換
    cameraPoints = newworldPoints*params.RotationMatrices(:,:,i)+params.TranslationVectors(i,:);
    All_cameraPoints = [All_cameraPoints;cameraPoints];
end
%%
%レーザーが作る平面z=ax+by+cの最小2乗平面を計算する
Sol = CalcLMSPlane(All_cameraPoints);
%%
%外れ値の検出とそれを省いた平面の式の計算
thr = 3;
Optimal_cameraPoints = [];
n = [Sol(1),Sol(2),-1];
p = [0,0,Sol(3)];
for i=1:size(All_cameraPoints,1)
    diffvec = All_cameraPoints(i,:)-p;
    dist = abs(dot(diffvec,n));
    if dist<thr
        Optimal_cameraPoints = [Optimal_cameraPoints;All_cameraPoints(i,:)];
    end
end
Sol2 = CalcLMSPlane(Optimal_cameraPoints);

%%
%レーザー平面と全レーザーのカメラ座標点のプロット
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
%高度計測の評価
Heights = [];
TrueHeights = [];
camOris = [];
erasestr = ["C:\Users\Mikihiro Ikura\Documents\GitHub\FisheyeCalibration\Photos\Laser_off\picure","mm.jpg"];
for i =1:11%今回は11枚分の画像が高度情報持ちの画像なので
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
%高度推定精度グラフの表示
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
%csvへカメラパラメータ群の出力
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
%csvへレーザー平面パラメータの出力
fprintf(fid,'%f,',Sol2);
fprintf(fid,'\n');
fclose(fid);

%%
%光切断法による高度計測の実践
%画像の読み込み
Ans_worlds = [];
for No = 1:11%ある程度明るくないと，輝度重心が計算できないでいる
    J = readimage(images_on,No);
    %輝度重心の計算
    [X,Y] = calcCoG(J);
    CoG_debug = [X,Y];
    %理想ピクセル座標系に変換
    IdealPixs = (CoG_debug-params.Intrinsics.DistortionCenter)*inv(params.Intrinsics.StretchMatrix);
    %カメラ座標系の直線の式の法線ベクトルの計算
    Lines = [];
    for i = 1:size(IdealPixs,1)
        u = IdealPixs(i,1);
        v = IdealPixs(i,2);
        ro = (u^2+v^2)^0.5;
        w = params.Intrinsics.MappingCoefficients(1)+params.Intrinsics.MappingCoefficients(2)*ro^2+params.Intrinsics.MappingCoefficients(3)*ro^3+params.Intrinsics.MappingCoefficients(4)*ro^4;
        Lines = [Lines;u,v,w];
    end
    %直線の式と平面からの求解@カメラ座標系
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
%光切断法とCalibration結果の高度計測のずれ
H_btwLS = Ans_worlds(:,3)-All_cameraPoints(:,3);
H_btwLS_outlier = filloutliers(H_btwLS,'linear');
Zure = max(H_btwLS_outlier)-min(H_btwLS_outlier);%見えている点でのずれなので，あまり意味ない値？
figure
plotnum =1:size(Ans_worlds,1);
plot(plotnum,H_btwLS);