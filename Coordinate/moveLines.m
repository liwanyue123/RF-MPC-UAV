
function [varargout] = moveLines(Lines,delatX,delatY,delatZ)

for k=1:nargout%Ҫ������ĸ���
      varargout{k} = moveLine2Plot(Lines(k) ,delatX,delatY,delatZ );
end

function [LineAfterMove] = moveLine2Plot(Line,delatX,delatY,delatZ)

%��һ�����ε��ĸ��㶼��ֱ��L��תtheta
LineAfterMove.a0 = movePlot(Line.a0,delatX,delatY,delatZ );
LineAfterMove.a1 = movePlot(Line.a1,delatX,delatY,delatZ );
 

