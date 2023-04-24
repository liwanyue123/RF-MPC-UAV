function [T] = composeTfromRandP(R,P)
%用R和p组合一个齐次变换矩阵 
  
assert(size(R,1) == 3 && size(R,2) == 3,"R Must have 3x3 matrix");
assert(size(P,1) == 3 && size(P,2) == 1,"P Must have 3x1 matrix");
T = [  R,P
      0,0,0,1];
 

end
