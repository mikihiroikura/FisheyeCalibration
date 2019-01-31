function camparam = All_Loff_cameraCalib(all_loff)
%calcCoG ���͉摜�̈ꕔ�̋P�x�d�S�̌v�Z
%   �ڍא����������ɋL�q
    files = [];
    for i =1:size(all_loff,2)
        files = [files;all_loff{i}.Files];
    end
    %%%%%%Camera calibration%%%%%
    %���[�U�[�Ȃ���Checkerboard�摜�Q���狁�߂�
    %�摜������Chekerboard�_�̌��o
    [imagePoints,boardSize] = detectCheckerboardPoints(files);
    squareSize = 50; % millimeters
    CheckboardworldPoints = generateCheckerboardPoints(boardSize,squareSize);
    I = readimage(all_loff{i},1); 
    imageSize = [size(I,1) size(I,2)];
    %calibration���ʂ̕ۑ�
    disp('calibration start');
    camparam = estimateFisheyeParameters(imagePoints,CheckboardworldPoints,imageSize);

end