function [R1] = calRotMatrix(axis,theta)%注意这里使用弧度制
global isSym
if(isSym)%符号计算
    R1=calRotMatrix_syms(axis,theta);
else
    R1= calRotMatrix_num(axis,theta);
end
end

function [R1] = calRotMatrix_num(axis,theta)%注意这里使用弧度制
%数值计算
%注意theta为空的时候

if isempty(theta)
    error="theta为空!!!!!!!"
end
c = cos(theta);
s = sin(theta);

Rx = [1,0,0,0;
    0,c,-s,0;
    0,s,c,0;
    0,0,0,1];

Ry = [c,0,s,0;
    0,1,0,0;
    -s,0,c,0;
    0,0,0,1];

Rz = [c,-s,0,0;
    s,c,0,0;
    0,0,1,0;
    0,0,0,1];

R1 = eye(4);

if axis == 'X'
    R1 = Rx;
elseif axis == 'Y'
    R1 = Ry;
elseif axis == 'Z'
    R1 = Rz;
end


end


function [R1] = calRotMatrix_syms(axis,theta)
%使用方法，先在外面定义好符号变量theta，之后才能调用这个s

if theta(1)=='-'
    t = sym(theta(2:end))*(-1);
else
    t = sym(theta);
end

c = sym(cos(t));
s = sym(sin(t));

Rx = [1,0,0,0;
    0,c,-s,0;
    0,s,c,0;
    0,0,0,1];

Ry = [c,0,s,0;
    0,1,0,0;
    -s,0,c,0;
    0,0,0,1];

Rz = [c,-s,0,0;
    s,c,0,0;
    0,0,1,0;
    0,0,0,1];

R1 = eye(4);

if axis == 'X'
    R1 = Rx;
elseif axis == 'Y'
    R1 = Ry;
elseif axis == 'Z'
    R1 = Rz;
end


end

