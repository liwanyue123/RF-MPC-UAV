function [V_end] = showCoordinateVec(C_body,Vec,color)
addpath(genpath("../TwistCalculation"));%������ʾ����ϵ���

R_world2Body= getCoordMatrix(C_body,'R');
V_end=R_world2Body*Vec;%�Ȱ�������������

Coord_positon = getCoordDot(C_body);%�õ��������ϵ����������ϵ�µ�λ��
lineArrow(Coord_positon,V_end,color );%������
end

