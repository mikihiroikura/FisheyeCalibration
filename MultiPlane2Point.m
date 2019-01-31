function point = MultiPlane2Point(all_plane)
%ï°êîÇÃïΩñ ÇÃéÆz=ax+by+cÇ©ÇÁç≈Ç‡ãﬂÇ¢ì_ÇÃç¿ïWÇãÅÇﬂÇÈ
    Sa2 = 0;
    Sab = 0;
    Sb2 = 0;
    Sa = 0;
    Sb = 0;
    S1 = 0;
    Sac = 0;
    Sbc = 0;
    Sc = 0;
    for i=1:size(all_plane,1)
        S = (all_plane(i,1)^2+all_plane(i,2)^2+1);
        Sa2 = Sa2 + all_plane(i,1)^2/S;
        Sb2 = Sb2 + all_plane(i,2)^2/S;
        Sab = Sab + all_plane(i,1)*all_plane(i,2)/S;
        Sa = Sa + all_plane(i,1)/S;
        Sb = Sb + all_plane(i,2)/S;
        S1 = S1 + 1/S;
        Sac = Sac + all_plane(i,1)*all_plane(i,3)/S;
        Sbc = Sbc + all_plane(i,2)*all_plane(i,3)/S;
        Sc = Sc + all_plane(i,3)/S;
    end
    A = [Sa2,Sab,-Sa;
         Sab,Sb2,-Sb;
         Sa,Sb,-S1];
    b = [-Sac;-Sbc;-Sc];
    point = A\b;
end