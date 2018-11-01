function [X,Y] = calcCoG(J)
%calcCoG 入力画像の一部の輝度重心の計算
%   詳細説明をここに記述
    BW = imbinarize(J,0.3);%0-1の間のパラメータでバイナリ化(0:0-1:255)
    BWsize = [size(BW,1),size(BW,2)];
    rows = [100 300];
    cols = [1 BWsize(2)];
    rect = [cols(1) rows(1) cols(2) rows(2)-rows(1)];
    % 2値化×元の画像にして，行ごとの輝度重心をとる
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

