function Optimal_linePoints = LineOptimization(dir,P,barPoints,thr)
    Optimal_linePoints = [];
    dir = dir/norm(dir);
    dir = dir';
    for i=1:size(barPoints,1)
        vect = barPoints(i,:)-P;
        dist = norm(vect-dot(vect,dir)*dir);
        if dist<thr
            Optimal_linePoints = [Optimal_linePoints;barPoints(i,:)];
        end
    end
end