function Optimal_cameraPoints = PlaneOptimization2(Sol,All_cameraPoints,thr)
%�ŏ�2�敽�ʂ̍œK���C�O��l����菜��
    Optimal_cameraPoints = [];
    n = [Sol(1),Sol(2),Sol(3)];
    p = [0,0,1/Sol(3)];
    for i=1:size(All_cameraPoints,1)
        diffvec = All_cameraPoints(i,:)-p;
        dist = abs(dot(diffvec,n));
        if dist<thr
            Optimal_cameraPoints = [Optimal_cameraPoints;All_cameraPoints(i,:)];
        end
    end
end