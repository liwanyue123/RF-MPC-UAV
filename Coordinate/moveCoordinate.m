function [C_AfterMove] = moveCoordinate(C,delatX,delatY,delatZ)

%��һ�����ε��ĸ��㶼��ֱ��L��תtheta
C_AfterMove.p0 = movePlot(C.p0,delatX,delatY,delatZ);
C_AfterMove.x1 = movePlot(C.x1,delatX,delatY,delatZ );
C_AfterMove.y1 = movePlot(C.y1,delatX,delatY,delatZ );
C_AfterMove.z1 = movePlot(C.z1,delatX,delatY,delatZ );
end

