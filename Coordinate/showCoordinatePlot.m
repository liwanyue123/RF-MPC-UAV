function [p_end] = showCoordinatePlot(C_Root,p,isShowLine)
% ��������p_end�ǿռ��в����ĵ㣬��������
%C_Root��һ������ϵ��p���������ϵ�е�Ŀ��������
%isShowLine��ʾ�Ƿ�Ҫ��������ߵ�����ϵԭ����
%x,y,z�ķ������� ���������Ƕ���
x_d=[C_Root.x1.X-C_Root.p0.X , C_Root.x1.Y-C_Root.p0.Y  ,C_Root.x1.Z-C_Root.p0.Z  ];
y_d=[C_Root.y1.X-C_Root.p0.X , C_Root.y1.Y-C_Root.p0.Y  ,C_Root.y1.Z-C_Root.p0.Z  ];
z_d=[C_Root.z1.X-C_Root.p0.X , C_Root.z1.Y-C_Root.p0.Y  ,C_Root.z1.Z-C_Root.p0.Z  ];

p_start=[C_Root.p0.X,C_Root.p0.Y,C_Root.p0.Z];
%Ŀ�������������ϵ�е�����
p_end=p_start+p(1)*x_d+p(2)*y_d+p(3)*z_d;%���ǽ�����ϵ������������Ͼ���
plot3(p_end(1),p_end(2),p_end(3),'o')

if(isShowLine)
    [u.a0]=deal(C_Root.p0);
    [u.a1.X,u.a1.Y,u.a1.Z]=deal(p_end(1),p_end(2),p_end(3) );
    showLine([u],'--','r')
end
p_end=p_end';

hold on;
end

