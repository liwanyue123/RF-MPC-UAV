function [mat4] = expandMatTo4Dim(mat3)
 mat4=eye(4);
 mat4(1:3,1:3)=mat3;
end

