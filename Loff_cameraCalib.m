function camparam = Loff_cameraCalib(loff)
%calcCoG 入力画像の一部の輝度重心の計算
%   詳細説明をここに記述
    %%%%%%Camera calibration%%%%%
    %レーザーなしのCheckerboard画像群から求める
    %画像中からChekerboard点の検出
    [imagePoints,boardSize] = detectCheckerboardPoints(loff.Files);
    squareSize = 50; % millimeters
    CheckboardworldPoints = generateCheckerboardPoints(boardSize,squareSize);
    I = readimage(loff,1); 
    imageSize = [size(I,1) size(I,2)];
    %calibration結果の保存
    disp('calibration start');
    camparam = estimateFisheyeParameters(imagePoints,CheckboardworldPoints,imageSize);

end