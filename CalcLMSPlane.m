function Sol = CalcLMSPlane(All_cameraPoints)
%UNTITLED2 この関数の概要をここに記述
%   詳細説明をここに記述
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
end

