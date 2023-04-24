function [T_world2Body] = getCoordMatrix(C_body,type)
% ����һ������ϵ��������������ϵ�����ı任����
%����֮ǰ��������ϵ��ʱ���Ǽ�¼������˵��λ�ã�����Ҫת��������Ҫ��һ��
Coord_p=[C_body.p0.X,C_body.p0.Y,C_body.p0.Z]';
Coord_x=[C_body.x1.X,C_body.x1.Y,C_body.x1.Z]'-Coord_p;
Coord_y=[C_body.y1.X,C_body.y1.Y,C_body.y1.Z]'-Coord_p;
Coord_z=[C_body.z1.X,C_body.z1.Y,C_body.z1.Z]'-Coord_p;

if(type=='T')
T_world2Body=[Coord_x,Coord_y,Coord_z,Coord_p;
    0,0,0,1];
elseif(type=='R')
    T_world2Body=[Coord_x,Coord_y,Coord_z];
else
    error("have not this type");
end
end

