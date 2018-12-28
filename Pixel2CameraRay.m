function Ray = Pixel2CameraRay(Point,params)
   %PixelÀ•WŒn‚©‚ç‚»‚ê‚ğ’Ê‚éCameraÀ•WŒn‚Ì’¼ü‚ğŒvZ‚·‚é
    IdealPixs = (Point-params.Intrinsics.DistortionCenter)*inv(params.Intrinsics.StretchMatrix);
    u = IdealPixs(1,1);
    v = IdealPixs(1,2);
    ro = (u^2+v^2)^0.5;
    w = params.Intrinsics.MappingCoefficients(1)+params.Intrinsics.MappingCoefficients(2)*ro^2+params.Intrinsics.MappingCoefficients(3)*ro^3+params.Intrinsics.MappingCoefficients(4)*ro^4;
    Ray = [u,v,w];
end