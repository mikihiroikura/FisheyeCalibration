function [X,Y] = calcCoG(J)
%calcCoG ���͉摜�̈ꕔ�̋P�x�d�S�̌v�Z
%   �ڍא����������ɋL�q
    BW = imbinarize(J,0.3);%0-1�̊Ԃ̃p�����[�^�Ńo�C�i����(0:0-1:255)
    BWsize = [size(BW,1),size(BW,2)];
    rows = [100 300];
    cols = [1 BWsize(2)];
    rect = [cols(1) rows(1) cols(2) rows(2)-rows(1)];
    % 2�l���~���̉摜�ɂ��āC�s���Ƃ̋P�x�d�S���Ƃ�
    BW = uint8(BW);
    J = J.*BW;
    colsvec = [1:BWsize(2)];
    colsvec = colsvec.';
    Jtrim = imcrop(J,rect);
    moments = double(Jtrim)*colsvec;
    mass = sum(Jtrim,2);
    Ytrim = find(moments);
    moments = moments(Ytrim);
    mass = mass(Ytrim);
    Y = double(Ytrim+99);
    X = moments./mass;
end

