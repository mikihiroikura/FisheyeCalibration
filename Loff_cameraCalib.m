function camparam = Loff_cameraCalib(loff)
%calcCoG ���͉摜�̈ꕔ�̋P�x�d�S�̌v�Z
%   �ڍא����������ɋL�q
    %%%%%%Camera calibration%%%%%
    %���[�U�[�Ȃ���Checkerboard�摜�Q���狁�߂�
    %�摜������Chekerboard�_�̌��o
    [imagePoints,boardSize] = detectCheckerboardPoints(loff.Files);
    squareSize = 50; % millimeters
    CheckboardworldPoints = generateCheckerboardPoints(boardSize,squareSize);
    I = readimage(loff,1); 
    imageSize = [size(I,1) size(I,2)];
    %calibration���ʂ̕ۑ�
    disp('calibration start');
    camparam = estimateFisheyeParameters(imagePoints,CheckboardworldPoints,imageSize);

end