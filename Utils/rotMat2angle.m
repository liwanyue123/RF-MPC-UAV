function [axis,angle] = rotMat2angle(R)
if(R(1,1)==1)
    axis="X";
    C=R(2,2);
    S=R(3,2);
    rad= atan2(S,C);
elseif(R(2,2)==1)
    axis="Y";
    C=R(1,1);
    S=R(1,3);
    rad= atan2(S,C);
elseif(R(3,3)==1)
    axis="Z";
    C=R(1,1);
    S=R(2,1);
    rad= atan2(S,C);
else
    error("这个不是绕单个轴旋转的矩阵")
end

angle=rad2angle(rad);
end

