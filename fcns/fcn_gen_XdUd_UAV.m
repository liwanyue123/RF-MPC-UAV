function [Xd,Ud] = fcn_gen_XdUd_UAV(t,Xt,p)
% Xt: current state [18,1] (position, velocity, attitude, and angular velocity of the robot)
% Ut: current control [4,1] (thrusts of four propellers)
% Xd: desired state [18,t]
% Ud: desired control [4,t]
%% parameters
gait = p.gait;
% desired velocity
acc_d = p.acc_d;
vel_d = p.vel_d;
yaw_d = p.yaw_d;

if acc_d==0
    acc_d=0.00001;
end

%% generate reference trajectory
% X = [pc dpc eta wb]
lent = length(t);
Xd = zeros(18,lent);
Ud = zeros(4,lent);

for ii = 1:lent
    if gait >= 0 % --- March forward and rotate ---
        %%%%%%%%%% linear motion %%%%%%%%%%%%%0
        pc_d = [0;0;p.z0];% initial position of the center of mass
        dpc_d = [0;0;0];% desired velocity of the center of mass

        % Generate reference trajectory for controlling the robot's linear motion
        % Uniform acceleration or uniform speed formula calculation
        for jj = 1:2
            if t(ii) < (vel_d(jj) / acc_d)
                dpc_d(jj) = acc_d * t(ii);
                pc_d(jj) = 1/2 * acc_d * t(ii)^2;
            else
                dpc_d(jj) = vel_d(jj);
                pc_d(jj) = vel_d(jj) * t(ii) - 1/2 * vel_d(jj) * vel_d(jj)/acc_d;
            end
        end
        %%%%%%%%%% angular motion %%%%%%%%%%%%%
        if isempty(Xt)% Initial value
            ea_d = [0;0;0];
        else
            ea_d = [0;0;yaw_d];
        end
        vR_d = reshape(expm(hatMap(ea_d)),[9,1]);
        wb_d = [0;0;0];% Angular velocity in the body coordinate system
    end
    Xd(:,ii) = [pc_d;dpc_d;vR_d;wb_d];

    %%%% force
    Ud(:, ii) = (p.mass * p.g / 4) * ones(4, 1);


end

end

