function E = CalcLSMPlane2(x,pointdata)
%UNTITLED7 ���̊֐��̊T�v�������ɋL�q
%   �ڍא����������ɋL�q
    E = sum((x(1).*pointdata(:,1)+x(2).*pointdata(:,2)+x(3).*pointdata(:,3)-1).^2./(x(1)^2+x(2)^2+x(3)^2));
end

