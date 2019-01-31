function Optimal_cameraPoints = PlaneOptimization2(Sol,All_cameraPoints,thr)
%最小2乗平面の最適化，外れ値を取り除く
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