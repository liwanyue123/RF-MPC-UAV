function [vec] = unHat( crossMat)
 
vec(1)=crossMat(3,2);
vec(2)=crossMat(1,3);
vec(3)=crossMat(2,1);
 
end

