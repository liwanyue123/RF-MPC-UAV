function [] = showCoodinate(C,name,rate,varargin)
%C.p0֮��Ķ�����ά��C.p0��x����㣬C.x1��x����յ�
% quiver3Ҫ�������λ�ã���������������
%rate�Ǳ���������1����̫���ˣ����ÿ�
hold on
%�������廻һ����ɫ
if nargin < 4
    % ���û�д�����ɫ������ʹ��Ĭ��ֵ
    text(C.p0.X,C.p0.Y,C.p0.Z,name,'color','r');
    quiver3(C.p0.X,  C.p0.Y,  C.p0.Z,   C.x1.X-C.p0.X,   C.x1.Y-C.p0.Y,   C.x1.Z-C.p0.Z  ,rate,'r')
    quiver3(C.p0.X,C.p0.Y,C.p0.Z,       C.y1.X-C.p0.X,C.y1.Y-C.p0.Y,C.y1.Z-C.p0.Z,rate,'g')
    quiver3(C.p0.X,C.p0.Y,C.p0.Z,       C.z1.X-C.p0.X,C.z1.Y-C.p0.Y,C.z1.Z-C.p0.Z,rate,'b')
    % ����������ǩ
    text(C.x1.X,   C.x1.Y,  C.x1.Z, 'x', 'color', 'r',  'FontSize', 14)
    text(C.y1.X,   C.y1.Y,  C.y1.Z,'y','color', 'g',   'FontSize', 14)
    text(C.z1.X,   C.z1.Y,  C.z1.Z,'z','color', 'b',   'FontSize', 14)
else
    color = varargin{1}; % ����ʹ�ô��ݵ���ɫ����
    text(C.p0.X,C.p0.Y,C.p0.Z,name,'color',color);
    quiver3(C.p0.X,  C.p0.Y,  C.p0.Z,   C.x1.X-C.p0.X,   C.x1.Y-C.p0.Y,   C.x1.Z-C.p0.Z  ,rate, 'color', color)
    quiver3(C.p0.X,C.p0.Y,C.p0.Z,       C.y1.X-C.p0.X,C.y1.Y-C.p0.Y,C.y1.Z-C.p0.Z,rate, 'color',color)
    quiver3(C.p0.X,C.p0.Y,C.p0.Z,       C.z1.X-C.p0.X,C.z1.Y-C.p0.Y,C.z1.Z-C.p0.Z,rate, 'color',color)
    
    % ����������ǩ
    text(C.x1.X,   C.x1.Y,  C.x1.Z, 'x', 'color', color,  'FontSize', 14)
    text(C.y1.X,   C.y1.Y,  C.y1.Z,'y','color', color,   'FontSize', 14)
    text(C.z1.X,   C.z1.Y,  C.z1.Z,'z','color', color,   'FontSize', 14)
end

end

