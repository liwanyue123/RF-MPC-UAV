function [C_after] =genCoordinateCoord(C_Root,R) 
%����������ϵ�±�ʾ������ϵC_Root������R����任���õ���������ϵ�±�ʾ������ϵC_after

%�Կռ�������λ�õ�����ϵC_RootΪ������Ȼ��������б任R���õ�����ռ��б任���C_after
%�����ǣ���һ����������ϵC_Base������ԭ����Ǹ���׼�����ᣬ����R�任��õ�һ����������
%Ȼ����C_RootΪ�������ѹ��������ʾ����
[p0.X,p0.Y,p0.Z]=deal(0,0,0);
[x1.X,x1.Y,x1.Z]=deal(1,0,0);%��ɫ��x
[y1.X,y1.Y,y1.Z]=deal(0,1,0);%��ɫ��y
[z1.X,z1.Y,z1.Z]=deal(0,0,1);%��ɫ��z
[C_Base.p0,C_Base.x1,C_Base.y1,C_Base.z1]=deal(p0,x1,y1,z1);%Coodinate

%C_Base����ԭ����Ǹ���׼������,C_Root�ǿռ��е�ǰ������
p=XYZ2p(C_Base.p0);
p(4)=1;
p_after= R*p';
a=p2XYZ(p_after(1:3));
a_end=calCoordinatePlot(C_Root,p_after(1:3) );
C_after.p0=p2XYZ(a_end);

p=XYZ2p(C_Base.x1);
p(4)=1;
p_after= R*p';
a=p2XYZ(p_after(1:3));
a_end=calCoordinatePlot(C_Root,p_after(1:3) );
C_after.x1=p2XYZ(a_end);

p=XYZ2p(C_Base.y1);
p(4)=1;
p_after= R*p';
a=p2XYZ(p_after(1:3));
a_end=calCoordinatePlot(C_Root,p_after(1:3) );
C_after.y1=p2XYZ(a_end);

p=XYZ2p(C_Base.z1);
p(4)=1;
p_after= R*p';
a=p2XYZ(p_after(1:3));
a_end=calCoordinatePlot(C_Root,p_after(1:3) );
C_after.z1=p2XYZ(a_end);
 

hold on;
 
 