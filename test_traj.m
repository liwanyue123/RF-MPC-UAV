clear; clc; close all;
addpath(genpath("Utils")); 
addpath(genpath("Coordinate")); 

t_final = 3;
num_points = 50;
p.z0=0.2;

t_ = linspace(0, t_final, num_points);
%% orientation
initial_R = eye(3);
middle_T = calRotMatrix('Y', deg2rad(-180))*calRotMatrix('Z', deg2rad(45));
middle_R = middle_T(1:3, 1:3);
final_T = calRotMatrix('Y', deg2rad(-360))*calRotMatrix('Z', deg2rad(90));
final_R = final_T(1:3, 1:3);

percent_rot=0.85;
% interpolation
t_normalized = (t_/t_final) * 2/ percent_rot; 
R_slerp1 = slerp(initial_R, middle_R, t_normalized); 
R_slerp2 = slerp(middle_R, final_R, t_normalized - 1); 
R_traj = zeros(3, 3, num_points);

for i = 1:num_points
    if t_normalized(i) <= 1
        R_traj(:, :, i) = R_slerp1{i};
    elseif t_normalized(i) <= 2
        R_traj(:, :, i) = R_slerp2{i};
    else
        R_traj(:, :, i) = final_R;
    end
end
%% position

% Parameters
radius = 6;
theta_start = 0;
theta_end =  5/6* pi;
% Generate the points
theta = linspace(theta_start, theta_end, num_points);
x = (radius * cos(theta)-radius);
z = 2*radius * sin(theta)+p.z0;
y = zeros(size(theta));
% Create the 1/2 circle trajectory
pos_traj = [x; y; z];
 

T_traj=zeros(4, 4, num_points);
T_traj(1:3,1:3,:)=R_traj;
T_traj(1:3,4,:)=pos_traj;


% draw
figure;
C_root=showWorldCoordinate(1);
 
for i=1:num_points
    Tw2b=T_traj(:,:,i);
    C_body=genCoordinateCoord(C_root,Tw2b);
showCoodinate(C_body,num2str(i),1)
end
 

xlabel('X');
ylabel('Y');
zlabel('Z');
title('UAV traj');


view(170, 30);
grid on;
axis equal;
 
%% interpolation function
 
function R_interp = slerp(R_initial, R_final, t_normalized)
% R_initial: (3x3)
% R_final: (3x3)
% t_normalized: range [0, 1]

num_timesteps = length(t_normalized);
R_interp = cell(1, num_timesteps);

for i = 1:num_timesteps
    R_interp{i} = R_initial * expm(t_normalized(i) * log_SO3(R_initial' * R_final));
end

end


function X = log_SO3(R)
% R: Rotation matrix (3x3)
theta = acos((trace(R) - 1) / 2); % Calculate rotation angle

if abs(theta) < 1e-12 % When the rotation angle is 0
X = zeros(3);
else
X = theta / (2 * sin(theta)) * (R - R'); % Calculate Lie algebra
end

end

