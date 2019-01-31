function [planesol,Optimal_cameraPoints,barPointsCam,All_barPixelPoints] = CameraPlaneBarCalibration(camparam,images_on,laser,bar,laserthr,barthr,cnt)
%laserOn,OFF�摜�Q����CLaser���ʂ̎��C�J�����p�����[�^�Q�̌v�Z
%�����F1�DLaserOff�摜�Q�C2�DLaserOn�摜�Q�C3�DLaser�s�͈́C4�DBar�s�͈�
%�ߒl�F1�DCamera�p�����[�^�C2�DLaser���ʂ̎�@Camera���W�n�C3�D�_���Laser�_�Q@Camera���W

%     %%%%%%Camera calibration%%%%%
%     %���[�U�[�Ȃ���Checkerboard�摜�Q���狁�߂�
%     %�摜������Chekerboard�_�̌��o
%     [imagePoints,boardSize] = detectCheckerboardPoints(images_off.Files);
%     squareSize = 50; % millimeters
%     CheckboardworldPoints = generateCheckerboardPoints(boardSize,squareSize);
%     I = readimage(images_off,1); 
%     imageSize = [size(I,1) size(I,2)];
%     %calibration���ʂ̕ۑ�
%     camparam = estimateFisheyeParameters(imagePoints,CheckboardworldPoints,imageSize);

    %%%%%���[�U�[�摜�Q���畽�ʌv�Z%%%%%
    %Camera���W�n�ł̃��[�U�[�_�Q�̕ۑ�
    %Pixel���W�n�ł̖_�ɉf�郌�[�U�[�_�̕ۑ�
    All_laserPointsCam = [];
    All_barPixelPoints = [];
    barPointsCam = [];
    for i = 1:size(images_on.Files,1)
        %�摜�̓ǂݍ���
        J = readimage(images_on,i);
        %%%%%%%%Checkerboard�ɂ�����Laser�̏���%%%%%%%%
        %�P�x�d�S�̌v�Z
        [X,Y] = calcCoG(J,laserthr,laser(1),laser(2));
        CoGPoints = [X,Y];
        %���[���h���W�n�ɕϊ�
        worldPoints = pointsToWorld(camparam.Intrinsics,camparam.RotationMatrices(:,:,i+cnt),camparam.TranslationVectors(i+cnt,:),CoGPoints);
        newworldPoints = [worldPoints,zeros(size(worldPoints,1),1)];
        %�J�������W�n�ɕϊ�
        cameraPoints = newworldPoints*camparam.RotationMatrices(:,:,i+cnt)+camparam.TranslationVectors(i+cnt,:);
        All_laserPointsCam = [All_laserPointsCam;cameraPoints];
        %%%%%�_���Laser�_�Q%%%%%
        %�_��Laser�_�Q@Pixel���W�̌v�Z
        [Xbar,Ybar] = calcCoG(J,barthr,bar(1),bar(2));
        BarPixelPoints = [Xbar,Ybar];
        All_barPixelPoints = [All_barPixelPoints;BarPixelPoints];
    end
    %���[�U�[�_�Q����ŏ�2�敽�ʂ��v�Z����
    %���[�U�[����镽��z=ax+by+c�̍ŏ�2�敽�ʂ��v�Z����
    Sol = CalcLMSPlane(All_laserPointsCam);
    %�O��l�̌��o�Ƃ�����Ȃ������ʂ̎��̌v�Z
    Optimal_cameraPoints = PlaneOptimization(Sol,All_laserPointsCam,3);
    planesol = CalcLMSPlane(Optimal_cameraPoints);
    planesol = transpose(planesol);
    %���[�U�[�摜�Q�ɉf��_�̏�̋P�_�Q���J�������W�n�Ɉڂ�
    %�_�Q���璼���̎����ŏ�2��ŋ��߂�
    for i=1:size(All_barPixelPoints,1)
        Line = Pixel2CameraRay(All_barPixelPoints(i,:),camparam);
        lamda = -planesol(3)/(planesol(1)*Line(1)+planesol(2)*Line(2)-Line(3));
        barPointsCam = [barPointsCam;lamda*Line];
    end
end