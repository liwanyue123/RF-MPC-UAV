function [C_Root] = showWorldCoordinate(rate)
%��������ϵ
[p0.X,p0.Y,p0.Z]=deal(0,0,0);%ԭ��
[x1.X,x1.Y,x1.Z]=deal(1,0,0);%��ɫ��x
[y1.X,y1.Y,y1.Z]=deal(0,1,0);%��ɫ��y
[z1.X,z1.Y,z1.Z]=deal(0,0,1);%��ɫ��z
[C_Root.p0,C_Root.x1,C_Root.y1,C_Root.z1]=deal(p0,x1,y1,z1);%����һ������ϵC_Root

 
showCoodinate(C_Root,"World",rate)


hold on;
end

