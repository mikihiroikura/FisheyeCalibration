function camparam = All_Loff_cameraCalib(all_loff)
%calcCoG 入力画像の一部の輝度重心の計算
%   詳細説明をここに記述
    files = [];
    for i =1:size(all_loff,2)
        files = [files;all_loff{i}.Files];
    end
    %%%%%%Camera calibration%%%%%
    %レーザーなしのCheckerboard画像群から求める
    %画像中からChekerboard点の検出
    [imagePoints,boardSize] = detectCheckerboardPoints(files);
    squareSize = 50; % millimeters
    CheckboardworldPoints = generateCheckerboardPoints(boardSize,squareSize);
    I = readimage(all_loff{i},1); 
    imageSize = [size(I,1) size(I,2)];
    %calibration結果の保存
    disp('calibration start');
    camparam = estimateFisheyeParameters(imagePoints,CheckboardworldPoints,imageSize);

end