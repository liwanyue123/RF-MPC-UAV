function [mat3] = compressMatTo3Dim(mat4)
mat3=zeros(3,3);
 mat3=mat4(1:3,1:3);
end

