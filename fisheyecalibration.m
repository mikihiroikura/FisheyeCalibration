%%
%画像データをまとめる
images = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\HighSpeedCamera\sample\PhotoCapture\Photos\Laser_off");

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
images_on = imageDatastore("C:\Users\Mikihiro Ikura\Documents\GitHub\HighSpeedCamera\sample\PhotoCapture\Photos\Laser_on");

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
%レーザー平面と全レーザーのカメラ座標点のプロット
f = figure;
graphX = linspace(min(Xcs),max(Xcs),500);
graphY = linspace(min(Ycs),max(Ycs),500);
[gX,gY] = meshgrid(graphX,graphY);
gZ = Sol(1)*gX+Sol(2)*gY+Sol(3);
mesh(gX,gY,gZ);
hold on
scatter3(Xcs,Ycs,Zcs);
%%
%高度計測の評価
Heights = [];
TrueHeights = [];
erasestr = ["C:\Users\Mikihiro Ikura\Documents\GitHub\HighSpeedCamera\sample\PhotoCapture\Photos\Laser_off\picure","mm.jpg"];
for i =1:15%今回は15枚分の画像が高度情報持ちの画像なので
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
%高度推定精度グラフの表示
f =figure;
no = 0:14;
plot(no,diffTrueHeights,'b-o');
hold on
plot(no,diffHeights,'r-o');
legend('True','Estimate');
g = figure;
plot(no,diffTrueHeights-diffHeights,'g-o');
legend('True-Estimate');

