function [] = showCoodinate(C,name,rate,varargin)
%C.p0之类的都是三维，C.p0是x轴起点，C.x1是x轴的终点
% quiver3要传入起点位置，和三个方向增量
%rate是比例，长度1可能太短了，不好看
hold on
%可以整体换一种颜色
if nargin < 4
    % 如果没有传递颜色参数，使用默认值
    text(C.p0.X,C.p0.Y,C.p0.Z,name,'color','r');
    quiver3(C.p0.X,  C.p0.Y,  C.p0.Z,   C.x1.X-C.p0.X,   C.x1.Y-C.p0.Y,   C.x1.Z-C.p0.Z  ,rate,'r')
    quiver3(C.p0.X,C.p0.Y,C.p0.Z,       C.y1.X-C.p0.X,C.y1.Y-C.p0.Y,C.y1.Z-C.p0.Z,rate,'g')
    quiver3(C.p0.X,C.p0.Y,C.p0.Z,       C.z1.X-C.p0.X,C.z1.Y-C.p0.Y,C.z1.Z-C.p0.Z,rate,'b')
    % 添加坐标轴标签
    text(C.x1.X,   C.x1.Y,  C.x1.Z, 'x', 'color', 'r',  'FontSize', 14)
    text(C.y1.X,   C.y1.Y,  C.y1.Z,'y','color', 'g',   'FontSize', 14)
    text(C.z1.X,   C.z1.Y,  C.z1.Z,'z','color', 'b',   'FontSize', 14)
else
    color = varargin{1}; % 否则使用传递的颜色参数
    text(C.p0.X,C.p0.Y,C.p0.Z,name,'color',color);
    quiver3(C.p0.X,  C.p0.Y,  C.p0.Z,   C.x1.X-C.p0.X,   C.x1.Y-C.p0.Y,   C.x1.Z-C.p0.Z  ,rate, 'color', color)
    quiver3(C.p0.X,C.p0.Y,C.p0.Z,       C.y1.X-C.p0.X,C.y1.Y-C.p0.Y,C.y1.Z-C.p0.Z,rate, 'color',color)
    quiver3(C.p0.X,C.p0.Y,C.p0.Z,       C.z1.X-C.p0.X,C.z1.Y-C.p0.Y,C.z1.Z-C.p0.Z,rate, 'color',color)
    
    % 添加坐标轴标签
    text(C.x1.X,   C.x1.Y,  C.x1.Z, 'x', 'color', color,  'FontSize', 14)
    text(C.y1.X,   C.y1.Y,  C.y1.Z,'y','color', color,   'FontSize', 14)
    text(C.z1.X,   C.z1.Y,  C.z1.Z,'z','color', color,   'FontSize', 14)
end

end

