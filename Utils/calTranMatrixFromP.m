function [R] = calTranMatrixFromP(P)
x=P(1);
y=P(2);
z=P(3);
%ÊýÖµ¼ÆËã 
R = [ 1,0,0,x;
      0,1,0,y;
      0,0,1,z;
      0,0,0,1];
 

end
