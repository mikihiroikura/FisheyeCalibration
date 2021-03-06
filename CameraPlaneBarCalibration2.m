function [planesol,Optimal_cameraPoints,barPointsCam,All_barPixelPoints] = CameraPlaneBarCalibration2(camparam,images_on,laser,bar,laserthr,barthr,cnt)
%laserOn,OFF画像群から，Laser平面の式，カメラパラメータ群の計算
%引数：1．LaserOff画像群，2．LaserOn画像群，3．Laser行範囲，4．Bar行範囲
%戻値：1．Cameraパラメータ，2．Laser平面の式@Camera座標系，3．棒上のLaser点群@Camera座標

%     %%%%%%Camera calibration%%%%%
%     %レーザーなしのCheckerboard画像群から求める
%     %画像中からChekerboard点の検出
%     [imagePoints,boardSize] = detectCheckerboardPoints(images_off.Files);
%     squareSize = 50; % millimeters
%     CheckboardworldPoints = generateCheckerboardPoints(boardSize,squareSize);
%     I = readimage(images_off,1); 
%     imageSize = [size(I,1) size(I,2)];
%     %calibration結果の保存
%     camparam = estimateFisheyeParameters(imagePoints,CheckboardworldPoints,imageSize);

    %%%%%レーザー画像群から平面計算%%%%%
    %Camera座標系でのレーザー点群の保存
    %Pixel座標系での棒に映るレーザー点の保存
    All_laserPointsCam = [];
    All_barPixelPoints = [];
    barPointsCam = [];
    for i = 1:size(images_on.Files,1)
        %画像の読み込み
        J = readimage(images_on,i);
        %%%%%%%%CheckerboardにあたるLaserの処理%%%%%%%%
        %輝度重心の計算
        [X,Y] = calcCoG(J,laserthr,laser(1),laser(2));
        CoGPoints = [X,Y];
        %ワールド座標系に変換
        worldPoints = pointsToWorld(camparam.Intrinsics,camparam.RotationMatrices(:,:,i+cnt),camparam.TranslationVectors(i+cnt,:),CoGPoints);
        newworldPoints = [worldPoints,zeros(size(worldPoints,1),1)];
        %カメラ座標系に変換
        cameraPoints = newworldPoints*camparam.RotationMatrices(:,:,i+cnt)+camparam.TranslationVectors(i+cnt,:);
        All_laserPointsCam = [All_laserPointsCam;cameraPoints];
        %%%%%棒上のLaser点群%%%%%
        %棒上Laser点群@Pixel座標の計算
        [Xbar,Ybar] = calcCoG(J,barthr,bar(1),bar(2));
        BarPixelPoints = [Xbar,Ybar];
        All_barPixelPoints = [All_barPixelPoints;BarPixelPoints];
    end
    %レーザー点群から最小2乗平面を計算する
    %レーザーが作る平面z=ax+by+cの最小2乗平面を計算する
    func = @(param)CalcLSMPlane2(param,All_laserPointsCam);
    flg = 1;
    false_cnt = 1;
    while flg
        x0 = -rand(1,3)*0.01*false_cnt;
        options = optimset('Display','iter','PlotFcns',@optimplotfval,'MaxFunEvals',1000);
        [Sol,fval,exitflag,output] = fminsearch(func,x0,options);
        if exitflag==1&&fval<3000
            flg = 0;
            false_cnt = 1;
        else
            false_cnt = false_cnt +1;
        end
        if false_cnt >10
            break;
        end
        
    end
    Optimal_cameraPoints = All_laserPointsCam;
%     Optimal_cameraPoints = PlaneOptimization2(Sol,All_laserPointsCam,3);
%     func2 = @(param)CalcLSMPlane2(param,Optimal_cameraPoints);
%     x0 = zeros(1,3);
%     planesol = fminsearch(func2,x0);
    planesol = Sol;
    %レーザー画像群に映る棒の上の輝点群をカメラ座標系に移す
    %点群から直線の式を最小2乗で求める
    for i=1:size(All_barPixelPoints,1)
        Line = Pixel2CameraRay(All_barPixelPoints(i,:),camparam);
        lamda = -planesol(3)/(planesol(1)*Line(1)+planesol(2)*Line(2)-Line(3));
        barPointsCam = [barPointsCam;lamda*Line];
    end
end