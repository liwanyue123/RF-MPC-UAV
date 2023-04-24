

function [varargout] = rotateRect(Rect,axis,theta)
 

for k=1:nargout
      varargout{k} =rotateRect4Plot(Rect(k) ,axis.a0,axis.a1,theta );
end


function [RectAfterRot] = rotateRect4Plot(Rect,L0,L1,theta)

%��һ�����ε��ĸ��㶼��ֱ��L��תtheta
RectAfterRot.a1 = rotate(Rect.a1,L0,L1,theta );
RectAfterRot.a2 = rotate(Rect.a2,L0,L1,theta );
RectAfterRot.a3 = rotate(Rect.a3,L0,L1,theta );
RectAfterRot.a4 = rotate(Rect.a4,L0,L1,theta );


