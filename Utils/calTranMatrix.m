function [R] = calTranMatrix(x,y,z)
global isSym
if(isSym)%符号计算
    R=calTranMatrix_sym (x,y,z);
else
    R= calTranMatrix_num(x,y,z);
end
end

function [R] = calTranMatrix_num(x,y,z)
%数值计算 
R = [ 1,0,0,x;
      0,1,0,y;
      0,0,1,z;
      0,0,0,1];
 

end

function [R] = calTranMatrix_sym (dis1,dis2,dis3)
%这个主要是为了符号运算生成的平移矩阵，
%x,y,z的偏移量传入
 
if dis1(1)=='-'
    x = sym(dis1(2:end))*(-1);
else
    x = sym(dis1);
end
if dis2(1)=='-'
    y = sym(dis2(2:end))*(-1);
else
    y = sym(dis2);
end
if dis3(1)=='-'
    z = sym(dis3(2:end))*(-1);
else
    z = sym(dis3);
end

R = [ 1,0,0,x;
    0,1,0,y;
    0,0,1,z;
    0,0,0,1];


end
